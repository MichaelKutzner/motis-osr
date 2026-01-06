#include "h3api.h.in"

#include "osr/location.h"
#include "osr/lookup.h"
#include "osr/point.h"
#include "osr/routing/parameters.h"
#include "osr/ways.h"

namespace osr {

constexpr auto const kH3Resolution = 11;

H3Index to_h3(point const&, int resolution);

vec<H3Index> isochrones_h3(profile_parameters const&, ways const& w,
                           lookup const& l, location const&, cost_t max_cost,
                           double max_matching_dist, int resolution,
                           bool nodes_only = true);

}  // namespace osr
