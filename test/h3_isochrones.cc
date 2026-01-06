#include "gtest/gtest.h"

#include <filesystem>

#include "h3api.h.in"

#include "utl/enumerate.h"

#include "osr/extract/extract.h"
#include "osr/location.h"
#include "osr/lookup.h"
#include "osr/routing/isochrones.h"
#include "osr/ways.h"

namespace fs = std::filesystem;

std::string load_data(std::string_view path) {
  auto const dir = fmt::format("/tmp/{}", path);
  auto ec = std::error_code{};
  fs::remove_all(dir, ec);
  fs::create_directories(dir, ec);

  osr::extract(false, path, dir, {});

  return dir;
}

TEST(routing, h3_isochrones) {
  auto const center = osr::point::from_latlng({49.872715, 8.651534});

  auto const isochrone = osr::to_h3(center, 11);

  EXPECT_EQ(H3Index{0x8B1FAE3ACCC0FFF}, isochrone);
}

TEST(isochrones, london_single_start) {
  auto const data_path = "test/london-corridor.osm.pbf";
  // auto const data_path = "test/Rheinufer.pbf";
  auto const start =
      // osr::location{51.1982061, 6.7363199, osr::level_t{0.F}};
      osr::location{51.54663831994142, -0.05622849779558692, osr::level_t{0.F}};

  auto const dir = load_data(data_path);
  auto const params = osr::foot<false>::parameters{};
  auto w = osr::ways{dir, cista::mmap::protection::READ};
  auto l = osr::lookup{w, dir, cista::mmap::protection::READ};

  auto const h3s =
      osr::isochrones_h3(params, w, l, start, osr::cost_t{160}, 5.0, 15, false);
  // osr::isochrones_h3(params, w, l, start, osr::cost_t{1800}, 11, false);
  // println("Before: {}", fmt::join(h3s, ", "));
  fmt::println("Formatting ...");
  for (auto const [i, idx] : utl::enumerate(h3s)) {
    if (i == 0) {
      fmt::print("{:X}", idx);
    } else {
      fmt::print(", {:X}", idx);
    }
  }
  fmt::println("\nDone.");
}

