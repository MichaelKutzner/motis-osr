#include "osr/routing/isochrones.h"

#include <concepts>

#include "utl/enumerate.h"
#include "utl/erase_duplicates.h"
#include "utl/pairwise.h"
#include "utl/pipes/iota.h"
#include "utl/pipes/transform.h"
#include "utl/pipes/vec.h"
#include "utl/zip.h"

#include "geo/rad_deg.h"

#include "fmt/format.h"

#include "osr/routing/dijkstra.h"
#include "osr/routing/parameters.h"
#include "osr/routing/profile.h"
#include "osr/routing/profiles/bike_sharing.h"
#include "osr/routing/profiles/car_sharing.h"
#include "osr/routing/profiles/foot.h"
#include "osr/types.h"

namespace osr {

H3Index to_h3(point const& p, int resolution) {
  auto const latlng = LatLng{geo::to_rad(p.lat()), geo::to_rad(p.lng())};
  auto idx = H3Index{};
  latLngToCell(&latlng, resolution, &idx);
  return idx;
}

template <bool IsWheelchair, typename Tracking>
void set_start(dijkstra<foot<IsWheelchair, Tracking>>& d,
               ways const& w,
               node_idx_t const start,
               cost_t cost,
               level_t const lvl) {
  d.add_start(
      w, typename foot<IsWheelchair, Tracking>::label{
             typename foot<IsWheelchair, Tracking>::node{start, lvl}, cost});
}

template <bike_costing Costing, unsigned int UpCost, unsigned int Exp>
void set_start(dijkstra<bike<Costing, UpCost, Exp>>& d,
               ways const& w,
               node_idx_t const start,
               cost_t cost,
               level_t const) {
  d.add_start(w, typename bike<Costing, UpCost, Exp>::label{
                     typename bike<Costing, UpCost, Exp>::node{
                         start, direction::kForward},
                     cost});
  d.add_start(w, typename bike<Costing, UpCost, Exp>::label{
                     typename bike<Costing, UpCost, Exp>::node{
                         start, direction::kBackward},
                     cost});
}

void set_start(dijkstra<bike_sharing>& d,
               ways const& w,
               node_idx_t const start,
               cost_t cost,
               level_t const lvl) {
  d.add_start(
      w, typename bike_sharing::label{
             typename bike_sharing::node{start, bike_sharing::node_type{}, lvl},
             cost});
}

void set_start(dijkstra<car>& d,
               ways const& w,
               node_idx_t const start,
               cost_t cost,
               level_t const) {
  d.add_start(w, car::label{car::node{start, 0, direction::kForward}, cost});
  d.add_start(w, car::label{car::node{start, 0, direction::kBackward}, cost});
};

template <typename Tracking>
void set_start(dijkstra<car_sharing<Tracking>>& d,
               ways const& w,
               node_idx_t const start,
               cost_t cost,
               level_t const lvl) {
  d.add_start(w, typename car_sharing<Tracking>::label{
                     typename car_sharing<Tracking>::node{
                         start, typename car_sharing<Tracking>::node_type{},
                         lvl, direction::kForward, 0U},
                     cost});
  d.add_start(w, typename car_sharing<Tracking>::label{
                     typename car_sharing<Tracking>::node{
                         start, typename car_sharing<Tracking>::node_type{},
                         lvl, direction::kBackward, 0U},
                     cost});
};

template <bool IsWheelchair, bool UseParking>
void set_start(dijkstra<car_parking<IsWheelchair, UseParking>>& d,
               ways const& w,
               node_idx_t const start,
               cost_t cost,
               level_t const lvl) {
  d.add_start(
      w, typename car_parking<IsWheelchair, UseParking>::label{
             typename car_parking<IsWheelchair, UseParking>::node{
                 start,
                 typename car_parking<IsWheelchair, UseParking>::node_type{},
                 lvl, direction::kForward, 0U},
             cost});
  d.add_start(
      w, typename car_parking<IsWheelchair, UseParking>::label{
             typename car_parking<IsWheelchair, UseParking>::node{
                 start,
                 typename car_parking<IsWheelchair, UseParking>::node_type{},
                 lvl, direction::kBackward, 0U},
             cost});
};

template <typename T>
node_idx_t to_node_idx_t(T const&);

template <typename T>
  requires requires(T const& t) { std::same_as<decltype(t.n_), node_idx_t>; }
node_idx_t to_node_idx_t(T const& key) {
  return key.n_;
}

template <>
node_idx_t to_node_idx_t<node_idx_t>(node_idx_t const& n) {
  return n;
}

vec<node_idx_t> get_nodes(auto const& map) {
  auto nodes = vec<node_idx_t>{};
  nodes.reserve(static_cast<unsigned int>(map.size()));

  for (auto const [node, entry] : map) {
    nodes.push_back(to_node_idx_t(node));
  }
  utl::sort(nodes);

  return nodes;
}

node_idx_t next_node(ways::routing const& r,
                     way_idx_t way_idx,
                     unsigned short i) {
  auto const& nodes = r.way_nodes_[way_idx];
  return i + 1 < nodes.size() ? nodes[i + 1] : node_idx_t::invalid();
}

bool is_reachable(node_idx_t n, vec<node_idx_t> const& nodes) {
  // Reminder: nodes are sorted
  auto const lb = std::lower_bound(nodes.begin(), nodes.end(), n);
  return lb != nodes.end() && *lb == n;
}

void add_segments(vec<H3Index>& h3s,
                  auto const& polyline,
                  unsigned int const start,
                  unsigned int const end,
                  int const resolution) {
  auto const stilts = utl::iota(start, end + 1) |
                      utl::transform([&](auto const i) {
                        return to_h3(polyline[i], resolution);
                      }) |
                      utl::vec();

  auto length = std::int64_t{};
  auto line = vec<H3Index>{};

  for (auto const [a, b] : utl::pairwise(stilts)) {
    gridPathCellsSize(a, b, &length);
    line.resize(static_cast<decltype(line)::size_type>(length));

    gridPathCells(a, b, line.data());
    for (auto const cell : line) {
      h3s.push_back(cell);
    }
  }
}

std::pair<unsigned, unsigned> find_way_offsets(ways const& w,
                                               way_idx_t way_idx,
                                               unsigned short const segment) {
  auto const way_osm_nodes = w.way_osm_nodes_[way_idx];
  auto const way_nodes = w.r_->way_nodes_[way_idx];
  auto const find_next = [&](node_idx_t const n, unsigned offset) {
    auto const osm_node = w.node_to_osm_[n];
    while (way_osm_nodes[offset] != osm_node) {
      ++offset;
    }
    return offset;
  };
  auto start = find_next(way_nodes[0], 0U);
  auto end = find_next(way_nodes[1], start + 1);
  for (auto s = 0U; s < segment; ++s) {
    start = end;
    end = find_next(way_nodes[s + 2], start + 1);
  }
  return {start, end};
}

void add_nodes(vec<H3Index>& h3s,
               vec<node_idx_t> nodes,
               ways const& w,
               int const resolution) {
  for (auto const node_idx : nodes) {
    h3s.push_back(to_h3(w.get_node_pos(node_idx), resolution));
  }
}

void add_paths(vec<H3Index>& h3s,
               vec<node_idx_t> nodes,
               ways const& w,
               ways::routing const& r,
               int const resolution) {
  for (auto const node_idx : nodes) {
    for (auto const [way_idx, i] : utl::zip_unchecked(
             r.node_ways_[node_idx], r.node_in_way_idx_[node_idx])) {
      if (is_reachable(next_node(r, way_idx, i), nodes)) {
        auto const [start, end] = find_way_offsets(w, way_idx, i);
        add_segments(h3s, w.way_polylines_[way_idx], start, end, resolution);
      }
    }
  }
}

vec<H3Index> isochrones_h3(profile_parameters const& parameters,
                           ways const& w,
                           lookup const& l,
                           location const& loc,
                           cost_t const max_cost,
                           double const max_matching_dist,
                           int const resolution,
                           bool const nodes_only) {
  return isochrones_h3(parameters, w, l, {loc}, {0}, max_cost,
                       max_matching_dist, resolution, nodes_only);
}

vec<H3Index> isochrones_h3(profile_parameters const& parameters,
                           ways const& w,
                           lookup const& l,
                           std::vector<location> const& locs,
                           std::vector<cost_t> const& offset_costs,
                           cost_t const max_cost,
                           double const max_matching_dist,
                           int const resolution,
                           bool const nodes_only) {
  utl::verify(locs.size() == offset_costs.size(),
              "locs.size() != offset_costs.size()");
  return std::visit(
      [&](ProfileParameters auto const& params) -> vec<H3Index> {
        using P = std::remove_cvref_t<decltype(params)>::profile_t;
        auto d = dijkstra<P>{};
        d.reset(max_cost);
        for (auto const [loc, offset_cost] : utl::zip(locs, offset_costs)) {
          auto const starts = static_cast<match_t>(
              l.match<P>(params, loc, false, direction::kForward,
                         max_matching_dist, nullptr));
          for (auto const& start : starts) {
            if (start.left_.valid()) {
              set_start(d, w, start.left_.node_, offset_cost, start.left_.lvl_);
            }
            if (start.right_.valid()) {
              set_start(d, w, start.right_.node_, offset_cost,
                        start.right_.lvl_);
            }
          }
        }
        d.run(params, w, *w.r_, max_cost, nullptr, nullptr, nullptr,
              direction::kForward);
        auto const nodes = get_nodes(d.cost_);

        auto reachable = vec<H3Index>{};
        reachable.reserve(static_cast<unsigned int>(nodes.size()));

        if (nodes_only) {
          add_nodes(reachable, nodes, w, resolution);
        } else {
          add_paths(reachable, nodes, w, *w.r_, resolution);
        }

        utl::sort(reachable);
        utl::erase_duplicates(reachable);
        return reachable;
      },
      parameters);
}

std::string to_string(vec<H3Index> const& h3s) {
  auto ss = std::stringstream{};
  for (auto const [i, idx] : utl::enumerate(h3s)) {
    if (i != 0) {
      ss << ", ";
    }
    ss << fmt::format("{:X}", idx);
  }
  return ss.str();
}

}  // namespace osr
