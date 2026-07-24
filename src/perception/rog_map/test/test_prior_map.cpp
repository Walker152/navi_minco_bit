#include <gtest/gtest.h>

#include <rog_map/prior_map.hpp>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
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

PriorMapData makePriorMap(int width,
  int height,
  const std::vector<std::pair<int, int>> & occupied_cells)
{
  PriorMapData prior;
  prior.loaded = true;
  prior.width = width;
  prior.height = height;
  prior.resolution = 1.0;
  prior.occupied.assign(static_cast<size_t>(width) * static_cast<size_t>(height), 0U);
  for (const auto & cell : occupied_cells) {
    const int image_row = height - 1 - cell.second;
    const size_t index = static_cast<size_t>(image_row) * static_cast<size_t>(width) +
                         static_cast<size_t>(cell.first);
    prior.occupied[index] = 1U;
  }
  return prior;
}

std::vector<uint8_t> projectReference(const PriorMapData & prior,
  int width,
  int height,
  double resolution,
  double origin_x,
  double origin_y)
{
  std::vector<uint8_t> mask(
    static_cast<size_t>(width) * static_cast<size_t>(height), 1U);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const double rog_x = origin_x + (static_cast<double>(x) + 0.5) * resolution;
      const double rog_y = origin_y + (static_cast<double>(y) + 0.5) * resolution;
      double map_x = 0.0;
      double map_y = 0.0;
      transformPriorMapPoint(prior.fixed_transform, rog_x, rog_y, map_x, map_y);
      if (priorMapOccupied(prior, map_x, map_y)) {
        mask[static_cast<size_t>(y) * static_cast<size_t>(width) +
             static_cast<size_t>(x)] = 0U;
      }
    }
  }
  return mask;
}

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

TEST(PriorMap, InitializesFixedTransformOnlyOnce)
{
  PriorMapData prior = makePriorMap(2, 2, {});
  const PriorMapTransform2D first{1.0, 2.0, 0.25};
  const PriorMapTransform2D second{9.0, 8.0, 0.75};

  ASSERT_TRUE(initializePriorMapTransformOnce(prior, first));
  ASSERT_TRUE(initializePriorMapTransformOnce(prior, second));

  EXPECT_TRUE(prior.transform_ready);
  EXPECT_DOUBLE_EQ(prior.fixed_transform.tx, first.tx);
  EXPECT_DOUBLE_EQ(prior.fixed_transform.ty, first.ty);
  EXPECT_DOUBLE_EQ(prior.fixed_transform.yaw, first.yaw);
}

TEST(PriorMap, RejectsNonFiniteFixedTransform)
{
  PriorMapData prior = makePriorMap(2, 2, {});
  const PriorMapTransform2D invalid{
    std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0};

  EXPECT_FALSE(initializePriorMapTransformOnce(prior, invalid));
  EXPECT_FALSE(prior.transform_ready);
}

TEST(PriorMap, KeepsFullFreeBufferUntilTransformIsReady)
{
  PriorMapData prior = makePriorMap(3, 3, {{0, 0}});

  EXPECT_TRUE(refreshPriorMapProjectionCache(prior, 10, 20, 3, 2, 1.0, 10.0, 20.0));
  EXPECT_FALSE(prior.projection_cache_ready);
  EXPECT_EQ(prior.cached_mask, std::vector<uint8_t>(6U, 1U));
  EXPECT_EQ(prior.scratch_mask.size(), 6U);

  const auto before = prior.cached_mask;
  EXPECT_FALSE(refreshPriorMapProjectionCache(prior, 10, 20, 3, 2, 1.0, 10.0, 20.0));
  EXPECT_EQ(prior.cached_mask, before);
}

TEST(PriorMap, RefreshesCacheOnceForUnchangedWindow)
{
  PriorMapData prior = makePriorMap(4, 3, {{1, 0}, {2, 1}});
  ASSERT_TRUE(initializePriorMapTransformOnce(prior, PriorMapTransform2D{}));
  ASSERT_TRUE(refreshPriorMapProjectionCache(prior, 0, 0, 3, 2, 1.0, 0.0, 0.0));
  ASSERT_TRUE(prior.projection_cache_ready);
  ASSERT_EQ(prior.cached_mask, projectReference(prior, 3, 2, 1.0, 0.0, 0.0));

  const auto before = prior.cached_mask;
  EXPECT_FALSE(refreshPriorMapProjectionCache(prior, 0, 0, 3, 2, 1.0, 0.0, 0.0));
  EXPECT_EQ(prior.cached_mask, before);
}

TEST(PriorMap, MarksOutsidePriorProjectionReady)
{
  PriorMapData prior = makePriorMap(1, 1, {{0, 0}});
  ASSERT_TRUE(initializePriorMapTransformOnce(prior, PriorMapTransform2D{}));

  ASSERT_TRUE(refreshPriorMapProjectionCache(
    prior, 20, 20, 2, 2, 1.0, 20.0, 20.0));
  EXPECT_TRUE(prior.projection_cache_ready);
  EXPECT_EQ(prior.cached_mask, std::vector<uint8_t>(4U, 1U));
}

TEST(PriorMap, FastProjectionMatchesPublicOriginYawSemantics)
{
  PriorMapData prior = makePriorMap(4, 4, {{0, 0}, {1, 2}, {3, 1}});
  prior.origin_x = 2.0;
  prior.origin_y = -1.0;
  prior.origin_yaw = std::acos(-1.0) / 2.0;
  const PriorMapTransform2D transform{1.0, 2.0, -0.25};
  ASSERT_TRUE(initializePriorMapTransformOnce(prior, transform));

  ASSERT_TRUE(refreshPriorMapProjectionCache(
    prior, -2, -2, 5, 5, 1.0, -2.0, -2.0));
  EXPECT_EQ(prior.cached_mask, projectReference(prior, 5, 5, 1.0, -2.0, -2.0));
}

TEST(PriorMap, SlidingCacheMatchesFullProjection)
{
  PriorMapData prior = makePriorMap(
    8, 8, {{0, 0}, {1, 2}, {2, 1}, {3, 3}, {4, 2}, {5, 5}, {6, 4}});
  ASSERT_TRUE(initializePriorMapTransformOnce(prior, PriorMapTransform2D{}));

  struct Window
  {
    int min_x;
    int min_y;
  };
  const std::vector<Window> windows{
    {1, 1}, {2, 1}, {1, 1}, {1, 2}, {1, 1}, {2, 2}, {4, 3}, {20, 20}};
  for (const auto & window : windows) {
    ASSERT_TRUE(refreshPriorMapProjectionCache(
      prior, window.min_x, window.min_y, 3, 3, 1.0, window.min_x, window.min_y));
    EXPECT_EQ(prior.cached_mask,
      projectReference(prior, 3, 3, 1.0, window.min_x, window.min_y));
  }
}

TEST(PriorMap, SlidingNearIntegerLimitDoesNotOverflow)
{
  PriorMapData prior = makePriorMap(1, 1, {});
  ASSERT_TRUE(initializePriorMapTransformOnce(prior, PriorMapTransform2D{}));
  constexpr int kFirstMin = std::numeric_limits<int>::max() - 2;
  constexpr int kSecondMin = std::numeric_limits<int>::max() - 1;

  ASSERT_TRUE(refreshPriorMapProjectionCache(
    prior, kFirstMin, kFirstMin, 3, 3, 1.0, kFirstMin, kFirstMin));
  EXPECT_TRUE(refreshPriorMapProjectionCache(
    prior, kSecondMin, kSecondMin, 3, 3, 1.0, kSecondMin, kSecondMin));
  EXPECT_EQ(prior.cached_mask, std::vector<uint8_t>(9U, 1U));
}

TEST(PriorMap, FusesCachedPriorOnlyWhenEnabledAndReady)
{
  PriorMapData prior = makePriorMap(2, 2, {{1, 0}, {1, 1}});
  ASSERT_TRUE(initializePriorMapTransformOnce(prior, PriorMapTransform2D{}));
  ASSERT_TRUE(refreshPriorMapProjectionCache(prior, 0, 0, 2, 2, 1.0, 0.0, 0.0));
  const std::vector<uint8_t> dynamic_mask{1U, 0U, 0U, 1U};
  const std::vector<uint8_t> dynamic_values{0U, 254U, 254U, 0U};
  std::vector<uint8_t> fused_mask;
  std::vector<uint8_t> fused_values;

  fusePriorMapProjection(
    true, prior, dynamic_mask, dynamic_values, fused_mask, fused_values);
  EXPECT_EQ(fused_mask, (std::vector<uint8_t>{1U, 0U, 0U, 0U}));
  EXPECT_EQ(fused_values, (std::vector<uint8_t>{0U, 254U, 254U, 254U}));

  fusePriorMapProjection(
    false, prior, dynamic_mask, dynamic_values, fused_mask, fused_values);
  EXPECT_EQ(fused_mask, dynamic_mask);
  EXPECT_EQ(fused_values, dynamic_values);
}

TEST(PriorMap, CacheSizeMismatchFallsBackToDynamicResult)
{
  PriorMapData prior = makePriorMap(1, 1, {{0, 0}});
  prior.transform_ready = true;
  prior.projection_cache_ready = true;
  prior.cached_mask.clear();
  const std::vector<uint8_t> dynamic_mask{1U};
  const std::vector<uint8_t> dynamic_values{0U};
  std::vector<uint8_t> fused_mask;
  std::vector<uint8_t> fused_values;

  fusePriorMapProjection(
    true, prior, dynamic_mask, dynamic_values, fused_mask, fused_values);
  EXPECT_EQ(fused_mask, dynamic_mask);
  EXPECT_EQ(fused_values, dynamic_values);
}

TEST(PriorMap, RejectsMismatchedDynamicBuffers)
{
  PriorMapData prior;
  const std::vector<uint8_t> dynamic_mask{1U, 1U};
  const std::vector<uint8_t> dynamic_values{0U};
  std::vector<uint8_t> fused_mask{0U};
  std::vector<uint8_t> fused_values{254U};

  EXPECT_THROW(
    fusePriorMapProjection(
      false, prior, dynamic_mask, dynamic_values, fused_mask, fused_values),
    std::invalid_argument);
  EXPECT_TRUE(fused_mask.empty());
  EXPECT_TRUE(fused_values.empty());
}

}  // namespace
}  // namespace rog_map
