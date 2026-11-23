#include "cras_rviz_vision_msgs/keepout_zones.hpp"

#include <gtest/gtest.h>

#include <vector>

using cras_rviz_vision_msgs::KeepoutZones;

namespace {

KeepoutZones::Zone square(double x, double y, double size) {
  return {{x, y}, {x + size, y}, {x + size, y + size}, {x, y + size}};
}

KeepoutZones makeZones(std::vector<KeepoutZones::Zone> zones) {
  KeepoutZones zones_;
  zones_.set("map", std::move(zones));
  return zones_;
}

}  // namespace

TEST(KeepoutZones, KnowsNothingWithoutAnyZone) {
  const KeepoutZones zones;
  EXPECT_TRUE(zones.empty());
  EXPECT_EQ(zones.getFrame(), "");
  EXPECT_TRUE(zones.zonesAt(0., 0.).empty());
  // An outline which does not enclose an area cannot tell anything about a position.
  EXPECT_FALSE(KeepoutZones::inside({{0., 0.}, {1., 1.}}, .5, .5));
  EXPECT_FALSE(KeepoutZones::inside({}, .5, .5));
}

TEST(KeepoutZones, FindsTheZoneAPositionLiesIn) {
  const auto zones = makeZones({square(0., 0., 10.), square(20., 0., 10.)});
  ASSERT_EQ(zones.size(), 2U);
  EXPECT_EQ(zones.getFrame(), "map");

  EXPECT_EQ(zones.zonesAt(5., 5.), std::vector<int>({0}));
  EXPECT_EQ(zones.zonesAt(25., 5.), std::vector<int>({1}));
  EXPECT_TRUE(zones.zonesAt(15., 5.).empty());
  EXPECT_TRUE(zones.zonesAt(5., 15.).empty());
}

TEST(KeepoutZones, FindsAllTheZonesOfAnOverlappingPosition) {
  const auto zones = makeZones({square(0., 0., 10.), square(5., 5., 10.)});
  EXPECT_EQ(zones.zonesAt(7., 7.), (std::vector<int>({0, 1})));
  EXPECT_EQ(zones.zonesAt(2., 2.), std::vector<int>({0}));
}

TEST(KeepoutZones, CountsTheBorderOfAZoneAsBeingInside) {
  const auto zones = makeZones({square(0., 0., 10.)});
  EXPECT_EQ(zones.zonesAt(0., 5.), std::vector<int>({0}));  // on an edge
  EXPECT_EQ(zones.zonesAt(5., 10.), std::vector<int>({0}));  // on an edge, other side
  EXPECT_EQ(zones.zonesAt(10., 10.), std::vector<int>({0}));  // in a corner
  EXPECT_TRUE(zones.zonesAt(-.1, 5.).empty());  // just outside, and no longer on the border
}

TEST(KeepoutZones, HandlesConcaveZones) {
  // An L-shaped zone: the arm of the L is empty even though the corners around it are all inside.
  const auto zones = makeZones({{{0., 0.}, {10., 0.}, {10., 3.}, {3., 3.}, {3., 10.}, {0., 10.}}});
  EXPECT_EQ(zones.zonesAt(5., 1.), std::vector<int>({0}));
  EXPECT_EQ(zones.zonesAt(1., 5.), std::vector<int>({0}));
  EXPECT_TRUE(zones.zonesAt(7., 7.).empty());
  EXPECT_TRUE(zones.zonesAt(-1., 5.).empty());
}

TEST(KeepoutZones, HandlesAZoneWhichRepeatsItsFirstCorner) {
  auto outline = square(0., 0., 4.);
  outline.push_back(outline.front());  // the closed outline some tools publish
  const auto zones = makeZones({outline});
  EXPECT_EQ(zones.zonesAt(2., 2.), std::vector<int>({0}));
  EXPECT_EQ(zones.zonesAt(0., 0.), std::vector<int>({0}));
  EXPECT_TRUE(zones.zonesAt(3., 5.).empty());
}

TEST(KeepoutZones, DropsTheOutlinesWhichEncloseNoArea) {
  const auto zones = makeZones({square(0., 0., 10.), {{0., 0.}, {1., 1.}},
        {{0., 0.}, {1., 0.}, {2., 0.}}, {{0., 0.}, {0., 0.}, {0., 0.}}});
  // Only the square is left, and it keeps the number it had in the message it arrived with... which is the only zone
  // there is, so the numbers of the zones are the numbers of the usable ones.
  ASSERT_EQ(zones.size(), 1U);
  EXPECT_EQ(zones.zonesAt(5., 5.), std::vector<int>({0}));
}

TEST(KeepoutZones, TellsWhetherTheZonesChanged) {
  KeepoutZones zones;
  EXPECT_TRUE(zones.set("map", {square(0., 0., 10.)}));  // the first zones are always something new
  EXPECT_FALSE(zones.set("map", {square(0., 0., 10.)}));  // the publisher sends the same zones again
  // an outline which only repeats its first corner describes the very same zone
  auto repeated = square(0., 0., 10.);
  repeated.push_back(repeated.front());
  EXPECT_FALSE(zones.set("map", {repeated}));
  EXPECT_TRUE(zones.set("odom", {square(0., 0., 10.)}));  // the same zone in another frame
  EXPECT_TRUE(zones.set("odom", {square(1., 0., 10.)}));  // a zone which moved
  EXPECT_TRUE(zones.set("odom", {}));  // and no zones at all
  EXPECT_FALSE(zones.set("odom", {}));
}

TEST(KeepoutZones, ForgetsTheZonesWhenTheyAreCleared) {
  auto zones = makeZones({square(0., 0., 10.)});
  ASSERT_FALSE(zones.zonesAt(5., 5.).empty());
  zones.clear();
  EXPECT_TRUE(zones.empty());
  EXPECT_EQ(zones.getFrame(), "");
  EXPECT_TRUE(zones.zonesAt(5., 5.).empty());

  // and the zones are replaced, not appended to
  zones.set("map", {square(100., 100., 2.)});
  ASSERT_EQ(zones.size(), 1U);
  EXPECT_TRUE(zones.zonesAt(5., 5.).empty());
  EXPECT_EQ(zones.zonesAt(101., 101.), std::vector<int>({0}));
}
