#include <rog_map/query_adapter.hpp>

#include <gtest/gtest.h>

#include <limits>
#include <memory>

namespace rog_map {
namespace {

std::shared_ptr<MapSnapshot> makeSnapshot(bool with_distances)
{
  auto snapshot = std::make_shared<MapSnapshot>();
  snapshot->sequence = 7;
  snapshot->snapshot_sequence = 7;
  snapshot->projection_sequence = 3;
  snapshot->mask_sequence = 5;
  snapshot->field_sequence = 6;
  snapshot->stamp = 2.0;
  snapshot->field_stamp = 1.5;
  snapshot->width = 2;
  snapshot->height = 2;
  snapshot->resolution = 1.0;
  snapshot->origin_x = 0.0;
  snapshot->origin_y = 0.0;
  snapshot->values.assign(4, 0U);
  if (with_distances) {
    snapshot->distances.assign(4, 1.0);
  }
  return snapshot;
}

TEST(QueryAdapterStatus, RejectsNonfiniteInput)
{
  QueryAdapter adapter;
  adapter.update(makeSnapshot(true), nullptr);

  const auto result =
    adapter.query(Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0));

  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.status, QueryStatus::NONFINITE_INPUT);
}

TEST(QueryAdapterStatus, ReportsFieldUninitialized)
{
  QueryAdapter adapter;
  adapter.update(makeSnapshot(false), nullptr);

  const auto result = adapter.query(Eigen::Vector3d(0.25, 0.25, 0.0));

  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.status, QueryStatus::FIELD_UNINITIALIZED);
  EXPECT_EQ(result.projection_sequence, 3U);
  EXPECT_EQ(result.mask_sequence, 5U);
  EXPECT_EQ(result.field_sequence, 6U);
  EXPECT_EQ(result.snapshot_sequence, 7U);
  EXPECT_NEAR(result.field_age_ms, 500.0, 1.0e-9);
}

TEST(QueryAdapterStatus, ReportsOutOfMap)
{
  QueryAdapter adapter;
  adapter.update(makeSnapshot(true), nullptr);

  const auto result = adapter.query(Eigen::Vector3d(5.0, 5.0, 0.0));

  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.status, QueryStatus::OUT_OF_MAP);
}

TEST(QueryAdapterStatus, ReturnsOkForFiniteField)
{
  QueryAdapter adapter;
  adapter.update(makeSnapshot(true), nullptr);

  const auto result = adapter.query(Eigen::Vector3d(0.25, 0.25, 0.0));

  EXPECT_TRUE(result.ok);
  EXPECT_EQ(result.status, QueryStatus::OK);
  EXPECT_DOUBLE_EQ(result.distance, 1.0);
  EXPECT_TRUE(result.gradient.allFinite());
}

}  // namespace
}  // namespace rog_map
