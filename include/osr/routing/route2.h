#include "osr/routing/route.h"

#include "osr/routing/tracking.h"
#include "osr/routing/profiles/bike.h"
#include "osr/routing/profiles/bike_sharing.h"
#include "osr/routing/profiles/car_sharing.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/profiles/car_parking.h"
#include "osr/routing/profiles/foot.h"

namespace osr {

// Contains pointers to template functions
// Used to allow explicit template instantiation
template <IsProfile Profile>
struct route_helper {
  static constexpr auto const route_one_many = static_cast<std::vector<std::optional<path>> (*)(Profile const&, ways const&, lookup const&, location const&, std::vector<location> const&, cost_t, direction, double, bitvec<node_idx_t> const*, sharing_data const*, elevation_storage const*, std::function<bool(path const&)> const&)>(&route);
  static constexpr auto const route_one_one = static_cast<std::optional<path> (*)(Profile const&, ways const&, lookup const&, location const&, location const&, cost_t, direction, double, bitvec<node_idx_t> const*, sharing_data const*, elevation_storage const*, routing_algorithm)>(&route);
};

extern template struct route_helper<foot<false, noop_tracking>>;
extern template struct route_helper<foot<true, noop_tracking>>;
extern template struct route_helper<foot<false, elevator_tracking>>;
extern template struct route_helper<foot<true, elevator_tracking>>;
extern template struct route_helper<bike<bike_costing::kFast, kElevationNoCost>>;
extern template struct route_helper<bike<bike_costing::kSafe, kElevationNoCost>>;
extern template struct route_helper<bike<bike_costing::kFast, kElevationLowCost>>;
extern template struct route_helper<bike<bike_costing::kSafe, kElevationLowCost>>;
extern template struct route_helper<bike<bike_costing::kFast, kElevationHighCost>>;
extern template struct route_helper<bike<bike_costing::kSafe, kElevationHighCost>>;
extern template struct route_helper<bike_sharing>;
extern template struct route_helper<car>;
extern template struct route_helper<car_parking<true, true>>;
extern template struct route_helper<car_parking<false, true>>;
extern template struct route_helper<car_parking<true, false>>;
extern template struct route_helper<car_parking<false, false>>;
extern template struct route_helper<car_sharing<noop_tracking>>;
extern template struct route_helper<car_sharing<elevator_tracking>>;

}
