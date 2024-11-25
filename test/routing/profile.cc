#include "osr/routing/profile.h"

#include "osr/routing/profiles/bike.h"
#include "osr/routing/profiles/bike_sharing.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/profiles/car_parking.h"
#include "osr/routing/profiles/foot.h"

static_assert(osr::IsProfile<osr::foot<false>>);  // Foot
static_assert(osr::IsProfile<osr::foot<true>>);  // Wheelchair
static_assert(osr::IsProfile<osr::bike>);
static_assert(osr::IsProfile<osr::car>);
static_assert(osr::IsProfile<osr::car_parking<false>>);  // No wheelchair
static_assert(osr::IsProfile<osr::car_parking<true>>);  // Wheelchair
static_assert(osr::IsProfile<osr::bike_sharing>);
