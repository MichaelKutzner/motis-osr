#include "osr/preprocessing/elevation/dem_driver.h"

#include <limits>

namespace fs = std::filesystem;

namespace osr::preprocessing::elevation {

bool dem_driver::add_tile(fs::path const& path) {
  auto const ext = path.extension().string();
  if (ext != ".hdr") {
    return false;
  }

  auto const idx = static_cast<std::size_t>(tiles_.size());
  auto const& tile = tiles_.emplace_back(dem_tile{path});
  auto const box = tile.get_coord_box();
  auto const min = decltype(rtree_)::coord_t{box.min_lat_, box.min_lng_};
  auto const max = decltype(rtree_)::coord_t{box.max_lat_, box.max_lng_};
  rtree_.insert(min, max, idx);
  return true;
}

elevation_meters_t dem_driver::get(point const& point) const {
  auto const p = decltype(rtree_)::coord_t{static_cast<float>(point.lat()),
                                           static_cast<float>(point.lng())};
  auto elevation = elevation_meters_t::invalid();
  rtree_.search(p, p,
                [&](auto const&, auto const&, std::size_t const& tile_idx) {
                  elevation = tiles_[tile_idx].get(point);
                  return elevation != elevation_meters_t::invalid();
                });
  return elevation;
}

tile_idx_t dem_driver::tile_idx(point const& point) const {
  auto const p = decltype(rtree_)::coord_t{static_cast<float>(point.lat()),
                                           static_cast<float>(point.lng())};
  auto idx = tile_idx_t::invalid();
  rtree_.search(p, p,
                [&](auto const&, auto const&, std::size_t const& tile_idx) {
                  idx = tiles_[tile_idx].tile_idx(point);
                  if (idx == tile_idx_t::invalid()) {
                    return false;
                  }
                  idx.tile_idx_ = static_cast<tile_idx_t::data_t>(tile_idx);
                  return true;
                });
  return idx;
}

resolution dem_driver::max_resolution() const {
  auto res = resolution{.x_ = std::numeric_limits<double>::quiet_NaN(),
                         .y_ = std::numeric_limits<double>::quiet_NaN()};
  for (auto const& tile : tiles_) {
    auto const r = tile.max_resolution();
    res.update(r);
  }
  return res;
}

std::size_t dem_driver::n_tiles() const { return tiles_.size(); }

}  // namespace osr::preprocessing::elevation
