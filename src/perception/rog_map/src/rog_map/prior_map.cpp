#include <rog_map/prior_map.hpp>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>

namespace rog_map {

namespace {

std::string readPgmToken(std::istream & input)
{
  while (input) {
    const int next = input.peek();
    if (next == std::char_traits<char>::eof()) {
      return {};
    }
    if (std::isspace(static_cast<unsigned char>(next))) {
      input.get();
      continue;
    }
    if (next == '#') {
      input.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      continue;
    }
    break;
  }

  std::string token;
  while (input) {
    const int next = input.peek();
    if (next == std::char_traits<char>::eof() ||
        std::isspace(static_cast<unsigned char>(next)) || next == '#') {
      break;
    }
    token.push_back(static_cast<char>(input.get()));
  }
  return token;
}

int parsePgmInteger(const std::string & token, const std::string & field, const std::string & path)
{
  if (token.empty()) {
    throw std::runtime_error("[PriorMap] missing PGM " + field + " in '" + path + "'");
  }
  size_t parsed = 0;
  int value = 0;
  try {
    value = std::stoi(token, &parsed);
  } catch (const std::exception &) {
    throw std::runtime_error(
            "[PriorMap] invalid PGM " + field + " '" + token + "' in '" + path + "'");
  }
  if (parsed != token.size()) {
    throw std::runtime_error(
            "[PriorMap] invalid PGM " + field + " '" + token + "' in '" + path + "'");
  }
  return value;
}

std::vector<uint8_t> loadPgm(
  const std::string & path, int & width, int & height)
{
  std::ifstream input(path, std::ios::binary);
  if (!input.is_open()) {
    throw std::runtime_error("[PriorMap] cannot open PGM file '" + path + "'");
  }

  const std::string magic = readPgmToken(input);
  if (magic != "P5" && magic != "P2") {
    throw std::runtime_error(
            "[PriorMap] unsupported PGM format '" + magic + "' in '" + path +
            "' (expected P5 or P2)");
  }
  width = parsePgmInteger(readPgmToken(input), "width", path);
  height = parsePgmInteger(readPgmToken(input), "height", path);
  const int max_value = parsePgmInteger(readPgmToken(input), "maximum gray value", path);
  if (width <= 0 || height <= 0) {
    throw std::runtime_error("[PriorMap] PGM dimensions must be positive in '" + path + "'");
  }
  if (max_value <= 0 || max_value > 255) {
    throw std::runtime_error(
            "[PriorMap] PGM maximum gray value must be in [1, 255] in '" + path + "'");
  }
  const size_t pixel_count = static_cast<size_t>(width) * static_cast<size_t>(height);
  if (pixel_count / static_cast<size_t>(width) != static_cast<size_t>(height)) {
    throw std::runtime_error("[PriorMap] PGM dimensions overflow pixel count in '" + path + "'");
  }

  std::vector<uint8_t> pixels(pixel_count, 0U);
  auto normalize = [max_value](int value) -> uint8_t {
    return static_cast<uint8_t>(std::lround(
      static_cast<double>(value) * 255.0 / static_cast<double>(max_value)));
  };

  if (magic == "P5") {
    const int separator = input.get();
    if (separator == std::char_traits<char>::eof() ||
        !std::isspace(static_cast<unsigned char>(separator))) {
      throw std::runtime_error("[PriorMap] missing PGM raster separator in '" + path + "'");
    }
    if (separator == '\r' && input.peek() == '\n') {
      input.get();
    }
    std::vector<unsigned char> raw(pixel_count, 0U);
    input.read(reinterpret_cast<char *>(raw.data()), static_cast<std::streamsize>(pixel_count));
    if (input.gcount() != static_cast<std::streamsize>(pixel_count)) {
      throw std::runtime_error("[PriorMap] PGM pixel count is smaller than declared in '" + path + "'");
    }
    if (input.peek() != std::char_traits<char>::eof()) {
      throw std::runtime_error("[PriorMap] PGM pixel count is larger than declared in '" + path + "'");
    }
    for (size_t i = 0; i < pixel_count; ++i) {
      if (raw[i] > max_value) {
        throw std::runtime_error("[PriorMap] PGM pixel exceeds maximum gray value in '" + path + "'");
      }
      pixels[i] = normalize(raw[i]);
    }
  } else {
    for (size_t i = 0; i < pixel_count; ++i) {
      const int value = parsePgmInteger(readPgmToken(input), "pixel", path);
      if (value < 0 || value > max_value) {
        throw std::runtime_error("[PriorMap] PGM pixel is outside the declared gray range in '" + path + "'");
      }
      pixels[i] = normalize(value);
    }
    if (!readPgmToken(input).empty()) {
      throw std::runtime_error("[PriorMap] PGM pixel count is larger than declared in '" + path + "'");
    }
  }
  return pixels;
}

double requireFiniteScalar(const YAML::Node & root, const char * key, const std::string & path)
{
  if (!root[key] || !root[key].IsScalar()) {
    throw std::runtime_error(
            "[PriorMap] required YAML scalar '" + std::string(key) + "' is missing in '" + path + "'");
  }
  const double value = root[key].as<double>();
  if (!std::isfinite(value)) {
    throw std::runtime_error(
            "[PriorMap] YAML scalar '" + std::string(key) + "' must be finite in '" + path + "'");
  }
  return value;
}

}  // namespace

PriorMapData loadPriorMap(const std::string & yaml_path, const std::string & pgm_path)
{
  if (yaml_path.empty()) {
    throw std::runtime_error("[PriorMap] projection.prior_map.yaml_path must not be empty");
  }

  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception & error) {
    throw std::runtime_error(
            "[PriorMap] failed to load YAML '" + yaml_path + "': " + error.what());
  }

  PriorMapData prior;
  try {
    prior.resolution = requireFiniteScalar(root, "resolution", yaml_path);
    prior.occupied_thresh = requireFiniteScalar(root, "occupied_thresh", yaml_path);
    prior.free_thresh = requireFiniteScalar(root, "free_thresh", yaml_path);
    if (!root["origin"] || !root["origin"].IsSequence() || root["origin"].size() != 3U) {
      throw std::runtime_error(
              "[PriorMap] YAML 'origin' must contain [x, y, yaw] in '" + yaml_path + "'");
    }
    prior.origin_x = root["origin"][0].as<double>();
    prior.origin_y = root["origin"][1].as<double>();
    prior.origin_yaw = root["origin"][2].as<double>();
    if (!std::isfinite(prior.origin_x) || !std::isfinite(prior.origin_y) ||
        !std::isfinite(prior.origin_yaw)) {
      throw std::runtime_error("[PriorMap] YAML origin values must be finite in '" + yaml_path + "'");
    }
    if (!root["negate"] || !root["negate"].IsScalar()) {
      throw std::runtime_error("[PriorMap] required YAML scalar 'negate' is missing in '" + yaml_path + "'");
    }
    const int negate = root["negate"].as<int>();
    if (negate != 0 && negate != 1) {
      throw std::runtime_error("[PriorMap] YAML 'negate' must be 0 or 1 in '" + yaml_path + "'");
    }
    prior.negate = negate == 1;
  } catch (const YAML::Exception & error) {
    throw std::runtime_error(
            "[PriorMap] invalid YAML value in '" + yaml_path + "': " + error.what());
  }

  if (prior.resolution <= 0.0) {
    throw std::runtime_error("[PriorMap] YAML resolution must be positive in '" + yaml_path + "'");
  }
  if (prior.free_thresh < 0.0 || prior.free_thresh > 1.0 ||
      prior.occupied_thresh < 0.0 || prior.occupied_thresh > 1.0 ||
      prior.free_thresh >= prior.occupied_thresh) {
    throw std::runtime_error(
            "[PriorMap] YAML thresholds must satisfy 0 <= free_thresh < occupied_thresh <= 1 in '" +
            yaml_path + "'");
  }

  std::filesystem::path image_path;
  if (!pgm_path.empty()) {
    image_path = pgm_path;
  } else {
    if (!root["image"] || !root["image"].IsScalar()) {
      throw std::runtime_error(
              "[PriorMap] YAML 'image' is required when projection.prior_map.pgm_path is empty");
    }
    image_path = root["image"].as<std::string>();
    if (image_path.is_relative()) {
      image_path = std::filesystem::path(yaml_path).parent_path() / image_path;
    }
  }

  const std::vector<uint8_t> pixels =
    loadPgm(image_path.lexically_normal().string(), prior.width, prior.height);
  prior.occupied.resize(pixels.size(), 0U);
  for (size_t i = 0; i < pixels.size(); ++i) {
    const double gray = static_cast<double>(pixels[i]);
    const double occupancy = prior.negate ? gray / 255.0 : (255.0 - gray) / 255.0;
    prior.occupied[i] = occupancy > prior.occupied_thresh ? 1U : 0U;
  }
  prior.loaded = true;
  return prior;
}

bool priorMapOccupied(const PriorMapData & prior_map, double map_x, double map_y)
{
  if (!prior_map.loaded || prior_map.width <= 0 || prior_map.height <= 0 ||
      prior_map.resolution <= 0.0 ||
      prior_map.occupied.size() !=
        static_cast<size_t>(prior_map.width) * static_cast<size_t>(prior_map.height)) {
    return false;
  }

  const double dx = map_x - prior_map.origin_x;
  const double dy = map_y - prior_map.origin_y;
  const double c = std::cos(prior_map.origin_yaw);
  const double s = std::sin(prior_map.origin_yaw);
  const double local_x = c * dx + s * dy;
  const double local_y = -s * dx + c * dy;
  const int image_col = static_cast<int>(std::floor(local_x / prior_map.resolution));
  const int my_from_bottom = static_cast<int>(std::floor(local_y / prior_map.resolution));
  const int image_row = prior_map.height - 1 - my_from_bottom;
  if (image_col < 0 || image_col >= prior_map.width || image_row < 0 ||
      image_row >= prior_map.height) {
    return false;
  }
  const size_t index = static_cast<size_t>(image_row) * static_cast<size_t>(prior_map.width) +
                       static_cast<size_t>(image_col);
  return prior_map.occupied[index] != 0U;
}

void transformPriorMapPoint(
  const PriorMapTransform2D & transform, double rog_x, double rog_y, double & map_x, double & map_y)
{
  const double c = std::cos(transform.yaw);
  const double s = std::sin(transform.yaw);
  map_x = c * rog_x - s * rog_y + transform.tx;
  map_y = s * rog_x + c * rog_y + transform.ty;
}

void fusePriorMapProjection(const PriorMapData & prior_map,
  const PriorMapTransform2D * transform,
  int width,
  int height,
  double resolution,
  double origin_x,
  double origin_y,
  const std::vector<uint8_t> & dynamic_mask,
  const std::vector<uint8_t> & dynamic_values,
  std::vector<uint8_t> & fused_mask,
  std::vector<uint8_t> & fused_values,
  std::vector<uint8_t> * prior_mask)
{
  const size_t expected = static_cast<size_t>(std::max(0, width)) *
                          static_cast<size_t>(std::max(0, height));
  if (width <= 0 || height <= 0) {
    fused_mask.clear();
    fused_values.clear();
    if (prior_mask != nullptr) {
      prior_mask->clear();
    }
    return;
  }
  if (resolution <= 0.0 || dynamic_mask.size() != expected || dynamic_values.size() != expected) {
    throw std::invalid_argument("fusePriorMapProjection: invalid projection geometry or buffer size");
  }

  fused_mask = dynamic_mask;
  fused_values = dynamic_values;
  if (prior_mask != nullptr) {
    prior_mask->assign(expected, 1U);
  }
  if (!prior_map.loaded || transform == nullptr) {
    return;
  }

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const size_t index = static_cast<size_t>(y) * static_cast<size_t>(width) +
                           static_cast<size_t>(x);
      const double rog_x = origin_x + (static_cast<double>(x) + 0.5) * resolution;
      const double rog_y = origin_y + (static_cast<double>(y) + 0.5) * resolution;
      double map_x = 0.0;
      double map_y = 0.0;
      transformPriorMapPoint(*transform, rog_x, rog_y, map_x, map_y);
      if (!priorMapOccupied(prior_map, map_x, map_y)) {
        continue;
      }
      if (prior_mask != nullptr) {
        (*prior_mask)[index] = 0U;
      }
      fused_mask[index] = 0U;
      fused_values[index] = 254U;
    }
  }
}

}  // namespace rog_map
