#include "osr/preprocessing/elevation/dem_driver.h"

#include <limits>

#include "osr/elevation_storage.h"
#include "osr/preprocessing/elevation/shared.h"

namespace fs = std::filesystem;

namespace osr::preprocessing::elevation {

bool dem_driver::add_tile(fs::path const& path) {
  auto const ext = path.extension().string();
  if (ext != ".hdr") {
    return false;
  }

  auto tile = dem_tile{path};
  auto const box = tile.get_coord_box();
  auto const min = decltype(rtree_)::coord_t{box.min_lat_, box.min_lng_};
  auto const max = decltype(rtree_)::coord_t{box.max_lat_, box.max_lng_};
  rtree_.insert(min, max, static_cast<std::size_t>(tiles_.size()));
  tiles_.emplace_back(std::move(tile));
  return true;
}

::osr::elevation_t dem_driver::get(::osr::point const& point) const {
  auto const p = decltype(rtree_)::coord_t{static_cast<float>(point.lat()),
                                           static_cast<float>(point.lng())};
  auto elevation = NO_ELEVATION_DATA;
  rtree_.search(p, p,
                [&](auto const&, auto const&, std::size_t const& tile_idx) {
                  elevation = tiles_[tile_idx].get(point);
                  return elevation != NO_ELEVATION_DATA;
                });
  return elevation;
}

tile_idx_t dem_driver::tile_idx(::osr::point const& point) const {
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

step_size dem_driver::get_step_size() const {
  auto steps = step_size{.x_ = std::numeric_limits<double>::quiet_NaN(),
                         .y_ = std::numeric_limits<double>::quiet_NaN()};
  for (auto const& tile : tiles_) {
    auto const s = tile.get_step_size();
    if (std::isnan(steps.x_) || s.x_ < steps.x_) {
      steps.x_ = s.x_;
    }
    if (std::isnan(steps.y_) || s.y_ < steps.y_) {
      steps.y_ = s.y_;
    }
  }
  return steps;
}

elevation_tile_idx_t dem_driver::get_tile_idx(::osr::point const& point) const {
  auto const p = decltype(rtree_)::coord_t{static_cast<float>(point.lat()),
                                           static_cast<float>(point.lng())};
  auto idx = elevation_tile_idx_t::invalid();
  rtree_.search(p, p,
                [&](auto const&, auto const&, std::size_t const& tile_idx) {
                  idx = elevation_tile_idx_t{tile_idx};
                  return idx != elevation_tile_idx_t::invalid();
                });
  return idx;
}

elevation_tile_idx_t dem_driver::get_sub_tile_idx(
    ::osr::point const& point) const {
  return get_tile_idx(point) * kSubTileFactor;
}

std::size_t dem_driver::n_tiles() const { return tiles_.size(); }

}  // namespace osr::preprocessing::elevation
