#include "osr/routing/route.h"

#include "osr/routing/tracking.h"
#include "osr/routing/profiles/foot.h"

namespace osr {

// extern
// template
// std::vector<std::optional<path>> route(
//     foot<false, noop_tracking>pr,
//     ways const&,
//     lookup const&,
//     search_profile,
//     location const& from,
//     std::vector<location> const& to,
//     cost_t max,
//     direction,
//     double max_match_distance,
//     bitvec<node_idx_t> const* blocked,
//     sharing_data const* sharing,
//     elevation_storage const*,
//     std::function<bool(path const&)> const& do_reconstruct);
extern
template
std::vector<std::optional<path>> route(
    foot<false, noop_tracking>pr,
    ways const&,
    lookup const&,
    location const& from,
    std::vector<location> const& to,
    cost_t max,
    direction,
    double max_match_distance,
    bitvec<node_idx_t> const* blocked = nullptr,
    sharing_data const* sharing = nullptr,
    elevation_storage const* = nullptr,
    std::function<bool(path const&)> const& do_reconstruct = [](path const&) {
      return false;
    });

extern
template
std::optional<path> route(
    foot<false, noop_tracking>pr,
                          ways const&,
                          lookup const&,
                          location const& from,
                          location const& to,
                          cost_t max,
                          direction,
                          double max_match_distance,
                          bitvec<node_idx_t> const* blocked,
                          sharing_data const* sharing,
                          elevation_storage const*,
                          routing_algorithm
                          );
}

