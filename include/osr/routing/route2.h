#include "osr/routing/route.h"

#include "osr/routing/tracking.h"
#include "osr/routing/profiles/foot.h"

namespace osr {

extern template struct route_helper<foot<false, noop_tracking>>;
extern template struct route_helper<foot<true, noop_tracking>>;

// template <typename... Ts>
// struct TypeList;
//
// using supported_types = TypeList<
//     foot<false, noop_tracking>,
//     foot<true, noop_tracking>
// >;



// extern
// template
// std::vector<std::optional<path>> route(
//     // IsProfile auto const&,
//     foot<false, noop_tracking> const&,
//     ways const&,
//     lookup const&,
//     location const& from,
//     std::vector<location> const& to,
//     cost_t max,
//     direction,
//     double max_match_distance,
//     bitvec<node_idx_t> const* blocked = nullptr,
//     sharing_data const* sharing = nullptr,
//     elevation_storage const* = nullptr,
//     std::function<bool(path const&)> const& do_reconstruct = [](path const&) {
//       return false;
//     });
//
// extern
// template
// std::optional<path> route(
//     // IsProfile auto const&,
//     foot<false, noop_tracking> const&,
//                           ways const&,
//                           lookup const&,
//                           location const& from,
//                           location const& to,
//                           cost_t max,
//                           direction,
//                           double max_match_distance,
//                           bitvec<node_idx_t> const* blocked,
//                           sharing_data const* sharing,
//                           elevation_storage const*,
//                           routing_algorithm
//                           );
}

