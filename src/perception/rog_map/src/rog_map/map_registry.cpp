#include <rog_map/map_registry.hpp>

#include <mutex>

namespace rog_map {
namespace {
std::mutex g_registry_mutex;
std::weak_ptr<MapQueryInterface> g_map;
}  // namespace

void MapRegistry::set(const std::shared_ptr<MapQueryInterface> & map)
{
  std::lock_guard<std::mutex> lock(g_registry_mutex);
  g_map = map;
}

std::shared_ptr<MapQueryInterface> MapRegistry::get()
{
  std::lock_guard<std::mutex> lock(g_registry_mutex);
  return g_map.lock();
}

}  // namespace rog_map
