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

TEST(isochrones, rhein_multiple_starts) {
  auto const data_path = "test/Rheinufer.pbf";
  auto const starts = std::vector{
      osr::location{51.1982061, 6.7363199, osr::level_t{0.F}},
      osr::location{51.185979, 6.740737, osr::level_t{0.F}},
      osr::location{51.189419, 6.734576, osr::level_t{0.F}},  // included
      osr::location{51.196033, 6.733664, osr::level_t{0.F}},  // not included
  };
  auto const offset_costs =
      std::vector<osr::cost_t>{5U * 60U, 3U * 60U, 8U * 60U, 8U * 60U};

  auto const dir = load_data(data_path);
  auto const params = osr::foot<false>::parameters{};
  auto w = osr::ways{dir, cista::mmap::protection::READ};
  auto l = osr::lookup{w, dir, cista::mmap::protection::READ};

  auto const h3s = osr::isochrones_h3(params, w, l, starts, offset_costs,
                                      osr::cost_t{8U * 60U}, 5.0, 11, false);

  EXPECT_EQ(
      osr::to_string(h3s),
      "8B1FA57482C1FFF, 8B1FA57482C3FFF, 8B1FA5748A49FFF, 8B1FA5749490FFF, "
      "8B1FA5749491FFF, 8B1FA5749493FFF, 8B1FA5749494FFF, 8B1FA5749495FFF, "
      "8B1FA574949AFFF, 8B1FA574949EFFF, 8B1FA57494B2FFF, 8B1FA57494B6FFF, "
      "8B1FA5749C22FFF, 8B1FA5749C34FFF, 8B1FA5749C35FFF, 8B1FA5749D12FFF, "
      "8B1FA5749D13FFF, 8B1FA5749D16FFF, 8B1FA5749D1AFFF, 8B1FA5749D1EFFF, "
      "8B1FA5749D81FFF, 8B1FA5749D8AFFF, 8B1FA5749D8CFFF, 8B1FA5749D8EFFF, "
      "8B1FA5749D92FFF, 8B1FA5749D99FFF, 8B1FA5749D9DFFF, 8B1FA5749DA8FFF, "
      "8B1FA5749DA9FFF, 8B1FA5749DAAFFF, 8B1FA5749DABFFF, 8B1FA5749DACFFF, "
      "8B1FA5749DADFFF, 8B1FA5749DAEFFF, 8B1FA574B84DFFF, 8B1FA574B868FFF, "
      "8B1FA574B869FFF, 8B1FA574B86DFFF, 8B1FA574B969FFF, 8B1FA574BB30FFF, "
      "8B1FA574BB31FFF, 8B1FA574BB33FFF, 8B1FA574BB34FFF, 8B1FA574BB36FFF");
  EXPECT_EQ(h3s, (osr::vec<H3Index>{
                     0x8B1FA57482C1FFF, 0x8B1FA57482C3FFF, 0x8B1FA5748A49FFF,
                     0x8B1FA5749490FFF, 0x8B1FA5749491FFF, 0x8B1FA5749493FFF,
                     0x8B1FA5749494FFF, 0x8B1FA5749495FFF, 0x8B1FA574949AFFF,
                     0x8B1FA574949EFFF, 0x8B1FA57494B2FFF, 0x8B1FA57494B6FFF,
                     0x8B1FA5749C22FFF, 0x8B1FA5749C34FFF, 0x8B1FA5749C35FFF,
                     0x8B1FA5749D12FFF, 0x8B1FA5749D13FFF, 0x8B1FA5749D16FFF,
                     0x8B1FA5749D1AFFF, 0x8B1FA5749D1EFFF, 0x8B1FA5749D81FFF,
                     0x8B1FA5749D8AFFF, 0x8B1FA5749D8CFFF, 0x8B1FA5749D8EFFF,
                     0x8B1FA5749D92FFF, 0x8B1FA5749D99FFF, 0x8B1FA5749D9DFFF,
                     0x8B1FA5749DA8FFF, 0x8B1FA5749DA9FFF, 0x8B1FA5749DAAFFF,
                     0x8B1FA5749DABFFF, 0x8B1FA5749DACFFF, 0x8B1FA5749DADFFF,
                     0x8B1FA5749DAEFFF, 0x8B1FA574B84DFFF, 0x8B1FA574B868FFF,
                     0x8B1FA574B869FFF, 0x8B1FA574B86DFFF, 0x8B1FA574B969FFF,
                     0x8B1FA574BB30FFF, 0x8B1FA574BB31FFF, 0x8B1FA574BB33FFF,
                     0x8B1FA574BB34FFF, 0x8B1FA574BB36FFF,
                 }));
}
