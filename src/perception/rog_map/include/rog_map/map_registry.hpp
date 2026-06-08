#pragma once

#include <memory>

#include <rog_map/map_query_interface.hpp>

namespace rog_map {

class MapRegistry
{
public:
  static void set(const std::shared_ptr<MapQueryInterface> & map);
  static std::shared_ptr<MapQueryInterface> get();
};

}  // namespace rog_map
