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

size_t checkedCellCount(int width, int height)
{
  if (width <= 0 || height <= 0) {
    throw std::invalid_argument("PriorMap projection dimensions must be positive");
  }
  const size_t width_size = static_cast<size_t>(width);
  const size_t height_size = static_cast<size_t>(height);
  if (width_size > std::numeric_limits<size_t>::max() / height_size) {
    throw std::invalid_argument("PriorMap projection dimensions overflow cell count");
  }
  return width_size * height_size;
}

bool priorStorageValid(const PriorMapData & prior_map)
{
  if (!prior_map.loaded || prior_map.width <= 0 || prior_map.height <= 0 ||
      !std::isfinite(prior_map.resolution) || prior_map.resolution <= 0.0) {
    return false;
  }
  const size_t width = static_cast<size_t>(prior_map.width);
  const size_t height = static_cast<size_t>(prior_map.height);
  return width <= std::numeric_limits<size_t>::max() / height &&
         prior_map.occupied.size() == width * height;
}

bool priorMapOccupiedWithRotation(const PriorMapData & prior_map,
  double map_x,
  double map_y,
  double origin_yaw_cos,
  double origin_yaw_sin)
{
  if (!priorStorageValid(prior_map) || !std::isfinite(map_x) || !std::isfinite(map_y)) {
    return false;
  }

  const double dx = map_x - prior_map.origin_x;
  const double dy = map_y - prior_map.origin_y;
  const double local_x = origin_yaw_cos * dx + origin_yaw_sin * dy;
  const double local_y = -origin_yaw_sin * dx + origin_yaw_cos * dy;
  const double image_col_value = local_x / prior_map.resolution;
  const double map_row_value = local_y / prior_map.resolution;
  if (!std::isfinite(image_col_value) || !std::isfinite(map_row_value) ||
      image_col_value < 0.0 || image_col_value >= static_cast<double>(prior_map.width) ||
      map_row_value < 0.0 || map_row_value >= static_cast<double>(prior_map.height)) {
    return false;
  }

  const int image_col = static_cast<int>(std::floor(image_col_value));
  const int image_row =
    prior_map.height - 1 - static_cast<int>(std::floor(map_row_value));
  const size_t index = static_cast<size_t>(image_row) * static_cast<size_t>(prior_map.width) +
                       static_cast<size_t>(image_col);
  return prior_map.occupied[index] != 0U;
}

bool samplePriorAtRogPointFast(
  const PriorMapData & prior_map, double rog_x, double rog_y)
{
  if (!prior_map.transform_ready) {
    return false;
  }
  const double map_x = prior_map.fixed_transform_cos * rog_x -
                       prior_map.fixed_transform_sin * rog_y +
                       prior_map.fixed_transform.tx;
  const double map_y = prior_map.fixed_transform_sin * rog_x +
                       prior_map.fixed_transform_cos * rog_y +
                       prior_map.fixed_transform.ty;
  return priorMapOccupiedWithRotation(prior_map,
    map_x,
    map_y,
    prior_map.fast_origin_yaw_cos,
    prior_map.fast_origin_yaw_sin);
}

bool sameProjectionGeometry(const PriorMapData & prior_map,
  int min_global_x,
  int min_global_y,
  int width,
  int height,
  double resolution,
  size_t expected_size)
{
  return prior_map.cached_min_x == min_global_x &&
         prior_map.cached_min_y == min_global_y &&
         prior_map.cached_width == width &&
         prior_map.cached_height == height &&
         std::abs(prior_map.cached_resolution - resolution) <= 1.0e-9 &&
         prior_map.cached_mask.size() == expected_size;
}

void saveProjectionGeometry(PriorMapData & prior_map,
  int min_global_x,
  int min_global_y,
  int width,
  int height,
  double resolution)
{
  prior_map.cached_min_x = min_global_x;
  prior_map.cached_min_y = min_global_y;
  prior_map.cached_width = width;
  prior_map.cached_height = height;
  prior_map.cached_resolution = resolution;
}

void rebuildPriorProjection(PriorMapData & prior_map,
  int min_global_x,
  int min_global_y,
  int width,
  int height,
  double resolution,
  double origin_x,
  double origin_y,
  size_t expected_size)
{
  prior_map.cached_mask.assign(expected_size, 1U);
  prior_map.scratch_mask.resize(expected_size);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const double rog_x = origin_x + (static_cast<double>(x) + 0.5) * resolution;
      const double rog_y = origin_y + (static_cast<double>(y) + 0.5) * resolution;
      if (samplePriorAtRogPointFast(prior_map, rog_x, rog_y)) {
        prior_map.cached_mask[static_cast<size_t>(y) * static_cast<size_t>(width) +
                              static_cast<size_t>(x)] = 0U;
      }
    }
  }
  saveProjectionGeometry(
    prior_map, min_global_x, min_global_y, width, height, resolution);
  prior_map.projection_cache_ready = true;
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
  const double c = std::cos(prior_map.origin_yaw);
  const double s = std::sin(prior_map.origin_yaw);
  return priorMapOccupiedWithRotation(prior_map, map_x, map_y, c, s);
}

void transformPriorMapPoint(
  const PriorMapTransform2D & transform, double rog_x, double rog_y, double & map_x, double & map_y)
{
  const double c = std::cos(transform.yaw);
  const double s = std::sin(transform.yaw);
  map_x = c * rog_x - s * rog_y + transform.tx;
  map_y = s * rog_x + c * rog_y + transform.ty;
}

bool initializePriorMapTransformOnce(
  PriorMapData & prior_map, const PriorMapTransform2D & transform)
{
  if (!prior_map.loaded) {
    return false;
  }
  if (prior_map.transform_ready) {
    return true;
  }
  if (!std::isfinite(transform.tx) || !std::isfinite(transform.ty) ||
      !std::isfinite(transform.yaw) || !std::isfinite(prior_map.origin_yaw)) {
    return false;
  }

  prior_map.fixed_transform = transform;
  prior_map.fixed_transform_cos = std::cos(transform.yaw);
  prior_map.fixed_transform_sin = std::sin(transform.yaw);
  prior_map.fast_origin_yaw_cos = std::cos(prior_map.origin_yaw);
  prior_map.fast_origin_yaw_sin = std::sin(prior_map.origin_yaw);
  prior_map.transform_ready = true;
  return true;
}

bool refreshPriorMapProjectionCache(PriorMapData & prior_map,
  int min_global_x,
  int min_global_y,
  int width,
  int height,
  double resolution,
  double origin_x,
  double origin_y)
{
  const size_t expected_size = checkedCellCount(width, height);
  if (!std::isfinite(resolution) || resolution <= 0.0 ||
      !std::isfinite(origin_x) || !std::isfinite(origin_y)) {
    throw std::invalid_argument("PriorMap projection geometry must be finite and positive");
  }

  const bool same_geometry = sameProjectionGeometry(prior_map,
    min_global_x,
    min_global_y,
    width,
    height,
    resolution,
    expected_size);
  if (!prior_map.transform_ready) {
    if (same_geometry && !prior_map.projection_cache_ready) {
      return false;
    }
    prior_map.cached_mask.assign(expected_size, 1U);
    prior_map.scratch_mask.resize(expected_size);
    saveProjectionGeometry(
      prior_map, min_global_x, min_global_y, width, height, resolution);
    prior_map.projection_cache_ready = false;
    return true;
  }

  if (prior_map.projection_cache_ready && same_geometry) {
    return false;
  }

  const bool reusable_geometry =
    prior_map.projection_cache_ready &&
    prior_map.cached_width == width &&
    prior_map.cached_height == height &&
    std::abs(prior_map.cached_resolution - resolution) <= 1.0e-9 &&
    prior_map.cached_mask.size() == expected_size;
  if (!reusable_geometry) {
    rebuildPriorProjection(prior_map,
      min_global_x,
      min_global_y,
      width,
      height,
      resolution,
      origin_x,
      origin_y,
      expected_size);
    return true;
  }

  const int64_t old_min_x = prior_map.cached_min_x;
  const int64_t old_min_y = prior_map.cached_min_y;
  const int64_t new_min_x = min_global_x;
  const int64_t new_min_y = min_global_y;
  const int64_t old_max_x = old_min_x + static_cast<int64_t>(width) - 1;
  const int64_t old_max_y = old_min_y + static_cast<int64_t>(height) - 1;
  const int64_t new_max_x = new_min_x + static_cast<int64_t>(width) - 1;
  const int64_t new_max_y = new_min_y + static_cast<int64_t>(height) - 1;
  const int64_t overlap_min_x = std::max(old_min_x, new_min_x);
  const int64_t overlap_min_y = std::max(old_min_y, new_min_y);
  const int64_t overlap_max_x = std::min(old_max_x, new_max_x);
  const int64_t overlap_max_y = std::min(old_max_y, new_max_y);
  const bool has_overlap =
    overlap_min_x <= overlap_max_x && overlap_min_y <= overlap_max_y;
  if (!has_overlap) {
    rebuildPriorProjection(prior_map,
      min_global_x,
      min_global_y,
      width,
      height,
      resolution,
      origin_x,
      origin_y,
      expected_size);
    return true;
  }

  prior_map.scratch_mask.resize(expected_size);
  std::fill(prior_map.scratch_mask.begin(), prior_map.scratch_mask.end(), 1U);
  const size_t overlap_width =
    static_cast<size_t>(overlap_max_x - overlap_min_x + 1);
  for (int64_t global_y = overlap_min_y; global_y <= overlap_max_y; ++global_y) {
    const size_t old_y = static_cast<size_t>(global_y - old_min_y);
    const size_t new_y = static_cast<size_t>(global_y - new_min_y);
    const size_t old_x = static_cast<size_t>(overlap_min_x - old_min_x);
    const size_t new_x = static_cast<size_t>(overlap_min_x - new_min_x);
    std::copy_n(
      prior_map.cached_mask.begin() + old_y * static_cast<size_t>(width) + old_x,
      overlap_width,
      prior_map.scratch_mask.begin() + new_y * static_cast<size_t>(width) + new_x);
  }

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const int64_t global_x = new_min_x + x;
      const int64_t global_y = new_min_y + y;
      if (global_x >= overlap_min_x && global_x <= overlap_max_x &&
          global_y >= overlap_min_y && global_y <= overlap_max_y) {
        continue;
      }
      const double rog_x = origin_x + (static_cast<double>(x) + 0.5) * resolution;
      const double rog_y = origin_y + (static_cast<double>(y) + 0.5) * resolution;
      if (samplePriorAtRogPointFast(prior_map, rog_x, rog_y)) {
        prior_map.scratch_mask[static_cast<size_t>(y) * static_cast<size_t>(width) +
                               static_cast<size_t>(x)] = 0U;
      }
    }
  }

  prior_map.cached_mask.swap(prior_map.scratch_mask);
  saveProjectionGeometry(
    prior_map, min_global_x, min_global_y, width, height, resolution);
  return true;
}

void fusePriorMapProjection(bool prior_enabled,
  const PriorMapData & prior_map,
  const std::vector<uint8_t> & dynamic_mask,
  const std::vector<uint8_t> & dynamic_values,
  std::vector<uint8_t> & fused_mask,
  std::vector<uint8_t> & fused_values)
{
  if (dynamic_mask.size() != dynamic_values.size()) {
    fused_mask.clear();
    fused_values.clear();
    throw std::invalid_argument(
            "fusePriorMapProjection: dynamic mask and value sizes differ");
  }

  fused_mask = dynamic_mask;
  fused_values = dynamic_values;
  if (!prior_enabled || !prior_map.loaded || !prior_map.transform_ready ||
      !prior_map.projection_cache_ready ||
      prior_map.cached_mask.size() != dynamic_mask.size()) {
    return;
  }

  for (size_t index = 0; index < prior_map.cached_mask.size(); ++index) {
    if (prior_map.cached_mask[index] == 0U) {
      fused_mask[index] = 0U;
      fused_values[index] = 254U;
    }
  }
}

}  // namespace rog_map
