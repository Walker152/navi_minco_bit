#include <gtest/gtest.h>

#include <rog_map/prior_map.hpp>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

namespace rog_map {
namespace {

class TempMapFiles
{
public:
  TempMapFiles()
  {
    const auto suffix = std::chrono::steady_clock::now().time_since_epoch().count();
    dir_ = std::filesystem::temp_directory_path() /
           ("rog_map_prior_map_test_" + std::to_string(suffix));
    std::filesystem::create_directories(dir_);
  }

  ~TempMapFiles()
  {
    std::error_code error;
    std::filesystem::remove_all(dir_, error);
  }

  std::filesystem::path path(const std::string & name) const { return dir_ / name; }

  void writeYaml(const std::string & image,
    int negate = 0,
    double resolution = 1.0,
    const std::string & origin = "[0.0, 0.0, 0.0]") const
  {
    std::ofstream output(path("map.yaml"));
    output << "image: " << image << '\n'
           << "resolution: " << resolution << '\n'
           << "origin: " << origin << '\n'
           << "negate: " << negate << '\n'
           << "occupied_thresh: 0.65\n"
           << "free_thresh: 0.25\n";
  }

  void writeP5(const std::string & name,
    int width,
    int height,
    const std::vector<uint8_t> & pixels) const
  {
    std::ofstream output(path(name), std::ios::binary);
    output << "P5\n# prior map test\n" << width << ' ' << height << "\n255\n";
    output.write(reinterpret_cast<const char *>(pixels.data()),
      static_cast<std::streamsize>(pixels.size()));
  }

private:
  std::filesystem::path dir_;
};

TEST(PriorMap, LoadsP5AndUsesBottomLeftMapOrigin)
{
  TempMapFiles files;
  files.writeP5("map.pgm", 3, 3, {
    0U, 255U, 255U,
    255U, 127U, 255U,
    255U, 255U, 255U});
  files.writeYaml("map.pgm");

  const PriorMapData prior = loadPriorMap(files.path("map.yaml").string(), "");
  EXPECT_TRUE(prior.loaded);
  EXPECT_EQ(prior.width, 3);
  EXPECT_EQ(prior.height, 3);
  EXPECT_TRUE(priorMapOccupied(prior, 0.5, 2.5));
  EXPECT_FALSE(priorMapOccupied(prior, 0.5, 0.5));
  EXPECT_FALSE(priorMapOccupied(prior, 1.5, 1.5));
}

TEST(PriorMap, LoadsCommentedP2WithNegate)
{
  TempMapFiles files;
  {
    std::ofstream output(files.path("map.pgm"));
    output << "P2\n# dimensions\n3 1\n# range\n255\n255 0 127\n";
  }
  files.writeYaml("ignored.pgm", 1);

  const PriorMapData prior =
    loadPriorMap(files.path("map.yaml").string(), files.path("map.pgm").string());
  EXPECT_TRUE(priorMapOccupied(prior, 0.5, 0.5));
  EXPECT_FALSE(priorMapOccupied(prior, 1.5, 0.5));
  EXPECT_FALSE(priorMapOccupied(prior, 2.5, 0.5));
}

TEST(PriorMap, RejectsPgmPixelCountMismatch)
{
  TempMapFiles files;
  files.writeP5("map.pgm", 3, 3, std::vector<uint8_t>(8U, 255U));
  files.writeYaml("map.pgm");

  EXPECT_THROW(loadPriorMap(files.path("map.yaml").string(), ""), std::runtime_error);
}

TEST(PriorMap, RejectsInvalidPgmDimensionsAndMaximumValue)
{
  TempMapFiles files;
  files.writeYaml("map.pgm");
  {
    std::ofstream output(files.path("map.pgm"));
    output << "P2\n0 3\n255\n";
  }
  EXPECT_THROW(loadPriorMap(files.path("map.yaml").string(), ""), std::runtime_error);
  {
    std::ofstream output(files.path("map.pgm"));
    output << "P2\n1 1\n256\n0\n";
  }
  EXPECT_THROW(loadPriorMap(files.path("map.yaml").string(), ""), std::runtime_error);
}

TEST(PriorMap, LoadsResolutionOriginTranslationAndYaw)
{
  TempMapFiles files;
  files.writeP5("map.pgm", 1, 1, {0U});
  files.writeYaml("map.pgm", 0, 0.5, "[2.0, 3.0, 1.5707963267948966]");
  const PriorMapData prior = loadPriorMap(files.path("map.yaml").string(), "");

  EXPECT_DOUBLE_EQ(prior.resolution, 0.5);
  EXPECT_DOUBLE_EQ(prior.origin_x, 2.0);
  EXPECT_DOUBLE_EQ(prior.origin_y, 3.0);
  EXPECT_NEAR(prior.origin_yaw, std::acos(-1.0) / 2.0, 1.0e-12);
  EXPECT_TRUE(priorMapOccupied(prior, 1.75, 3.25));
  EXPECT_FALSE(priorMapOccupied(prior, 2.25, 3.25));
  EXPECT_FALSE(priorMapOccupied(prior, 10.0, 10.0));
}

TEST(PriorMap, AppliesRogToMapTransformInTargetSourceDirection)
{
  const PriorMapTransform2D transform{10.0, 20.0, std::acos(-1.0) / 2.0};
  double map_x = 0.0;
  double map_y = 0.0;
  transformPriorMapPoint(transform, 2.0, 3.0, map_x, map_y);
  EXPECT_NEAR(map_x, 7.0, 1.0e-9);
  EXPECT_NEAR(map_y, 22.0, 1.0e-9);
}

TEST(PriorMap, FusesOnlyPriorObstacles)
{
  PriorMapData prior;
  prior.loaded = true;
  prior.width = 2;
  prior.height = 2;
  prior.resolution = 1.0;
  prior.occupied = {0U, 1U, 0U, 1U};
  const PriorMapTransform2D identity;
  const std::vector<uint8_t> dynamic_mask{1U, 0U, 0U, 1U};
  const std::vector<uint8_t> dynamic_values{0U, 254U, 254U, 0U};
  std::vector<uint8_t> fused_mask;
  std::vector<uint8_t> fused_values;
  std::vector<uint8_t> prior_mask;

  fusePriorMapProjection(prior,
    &identity,
    2,
    2,
    1.0,
    0.0,
    0.0,
    dynamic_mask,
    dynamic_values,
    fused_mask,
    fused_values,
    &prior_mask);

  EXPECT_EQ(fused_mask, (std::vector<uint8_t>{1U, 0U, 0U, 0U}));
  EXPECT_EQ(fused_values, (std::vector<uint8_t>{0U, 254U, 254U, 254U}));
  EXPECT_EQ(prior_mask, (std::vector<uint8_t>{1U, 0U, 1U, 0U}));
}

TEST(PriorMap, MissingTransformRebuildsPureDynamicResult)
{
  PriorMapData prior;
  prior.loaded = true;
  prior.width = 1;
  prior.height = 1;
  prior.resolution = 1.0;
  prior.occupied = {1U};
  const PriorMapTransform2D identity;
  const std::vector<uint8_t> dynamic_mask{1U};
  const std::vector<uint8_t> dynamic_values{0U};
  std::vector<uint8_t> fused_mask;
  std::vector<uint8_t> fused_values;
  std::vector<uint8_t> prior_mask;

  fusePriorMapProjection(prior,
    &identity,
    1,
    1,
    1.0,
    0.0,
    0.0,
    dynamic_mask,
    dynamic_values,
    fused_mask,
    fused_values,
    &prior_mask);
  ASSERT_EQ(fused_mask, (std::vector<uint8_t>{0U}));
  ASSERT_EQ(prior_mask, (std::vector<uint8_t>{0U}));

  fusePriorMapProjection(prior,
    nullptr,
    1,
    1,
    1.0,
    0.0,
    0.0,
    dynamic_mask,
    dynamic_values,
    fused_mask,
    fused_values,
    &prior_mask);
  EXPECT_EQ(fused_mask, dynamic_mask);
  EXPECT_EQ(fused_values, dynamic_values);
  EXPECT_EQ(prior_mask, (std::vector<uint8_t>{1U}));
}

}  // namespace
}  // namespace rog_map
