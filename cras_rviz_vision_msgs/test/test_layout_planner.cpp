#include "cras_rviz_vision_msgs/layout_planner.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

using cras_rviz_vision_msgs::estimateLabelWidth;
using cras_rviz_vision_msgs::LayoutParams;
using cras_rviz_vision_msgs::LayoutItem;
using cras_rviz_vision_msgs::ScreenLayout;
using cras_rviz_vision_msgs::ScreenRect;

namespace {

//! Detection with an image preview of the given size at the given place of the view.
LayoutItem makeItem(double x, double y, double priority = 1., double width = 100., double height = 80.) {
  LayoutItem item;
  item.anchor = {x - 3., y - 3., 6., 6.};
  item.priority = priority;
  item.previewWidth = width;
  item.previewHeight = height;
  return item;
}

//! Number of detections of the given importance crowded to one place of the view.
std::vector<LayoutItem> makeCrowd(size_t count, double x = 300., double y = 300.) {
  std::vector<LayoutItem> items;
  for (size_t i = 0; i < count; ++i) {
    items.push_back(makeItem(x + static_cast<double>(i), y, 1.));
  }
  return items;
}

//! Area of the intersection of two rectangles.
double overlap(const ScreenRect& a, const ScreenRect& b) {
  const auto x = std::min(a.x + a.w, b.x + b.w) - std::max(a.x, b.x);
  const auto y = std::min(a.y + a.h, b.y + b.h) - std::max(a.y, b.y);
  return x > 0. && y > 0. ? x * y : 0.;
}

//! Number of the previews which overlap each other.
size_t overlappingPreviews(const std::vector<LayoutItem>& items) {
  size_t count = 0;
  for (size_t a = 0; a < items.size(); ++a) {
    if (!items[a].previewPlaced) {
      continue;
    }
    for (size_t b = a + 1; b < items.size(); ++b) {
      if (items[b].previewPlaced && overlap(items[a].preview, items[b].preview) > 1.) {
        ++count;
      }
    }
  }
  return count;
}

//! Number of the placed labels which overlap each other.
size_t overlappingLabels(const std::vector<LayoutItem>& items) {
  size_t count = 0;
  for (size_t a = 0; a < items.size(); ++a) {
    if (!items[a].labelPlaced) {
      continue;
    }
    for (size_t b = a + 1; b < items.size(); ++b) {
      if (items[b].labelPlaced && overlap(items[a].label, items[b].label) > 1.) {
        ++count;
      }
    }
  }
  return count;
}

//! Number of the placed annotations which are not visible in the view.
size_t hiddenAnnotations(const std::vector<LayoutItem>& items, double viewWidth, double viewHeight) {
  const ScreenRect view {0., 0., viewWidth, viewHeight};
  size_t count = 0;
  for (const auto& item : items) {
    for (const auto& annotation : {item.preview, item.label}) {
      if (annotation.area() > 0. && overlap(annotation, view) < .5 * annotation.area()) {
        ++count;
      }
    }
  }
  return count;
}

size_t previewCountIn(const LayoutParams& params, double viewWidth, double viewHeight, size_t crowd) {
  auto items = makeCrowd(crowd);
  ScreenLayout layout {viewWidth, viewHeight, params};
  layout.place(items);
  return layout.previewCount();
}

}  // namespace

TEST(ScreenLayout, SingleItemGetsItsPreviewAtItsDetection) {
  std::vector<LayoutItem> items {makeItem(500., 400.)};
  ScreenLayout layout {1000., 800., LayoutParams()};
  layout.place(items);

  ASSERT_EQ(1U, layout.previewCount());
  ASSERT_TRUE(items[0].previewPlaced);
  // The detection is in the middle of an empty view, so the preview can be shown right at it.
  EXPECT_NEAR(500., items[0].preview.x + .5 * items[0].preview.w, 1.);
  EXPECT_NEAR(400., items[0].preview.y + .5 * items[0].preview.h, 1.);
}

TEST(ScreenLayout, OverlappingDetectionsGetPreviewsSideBySide) {
  std::vector<LayoutItem> items {makeItem(500., 400.), makeItem(505., 402.)};
  ScreenLayout layout {1000., 800., LayoutParams()};
  layout.place(items);

  ASSERT_EQ(2U, layout.previewCount());
  EXPECT_EQ(0U, overlappingPreviews(items));
}

TEST(ScreenLayout, DenseClusterOfDetections) {
  // A crowd of people seen from a distance: a lot of detections within a few pixels of each other.
  auto params = LayoutParams();
  params.maxPreviews = 4;
  auto items = makeCrowd(12);
  for (size_t i = 0; i < items.size(); ++i) {
    items[i].priority = 1. + static_cast<double>(i) / 100.;
  }
  ScreenLayout layout {800., 600., params};
  layout.place(items);

  EXPECT_EQ(4U, layout.previewCount());
  EXPECT_EQ(0U, overlappingPreviews(items));
  size_t shown = 0;
  for (const auto& item : items) {
    shown += item.previewPlaced;
  }
  EXPECT_EQ(4U, shown);
}

TEST(ScreenLayout, MoreRoomInAZoomedInViewMeansMorePreviews) {
  auto params = LayoutParams();
  params.maxPreviews = 100;
  const auto zoomedOut = previewCountIn(params, 400., 300., 40);
  const auto zoomedIn = previewCountIn(params, 1600., 1200., 40);
  EXPECT_GT(zoomedOut, 0U);
  EXPECT_LT(zoomedOut, zoomedIn);
}

TEST(ScreenLayout, PreviewNearTheViewCornerIsMovedIntoTheViewButNotFurther) {
  std::vector<LayoutItem> items {makeItem(20., 20.), makeItem(500., 400.), makeItem(-400., 900.)};
  ScreenLayout layout {1000., 800., LayoutParams()};
  layout.place(items);

  // The preview of the detection at the corner does not fit there, so it is moved to the closest place which is
  // mostly visible - but not further than one preview size, otherwise it would not be clear which detection it belongs
  // to.
  EXPECT_EQ(2U, layout.previewCount());
  ASSERT_TRUE(items[0].previewPlaced);
  const auto centreX = items[0].preview.x + .5 * items[0].previewWidth;
  const auto centreY = items[0].preview.y + .5 * items[0].previewHeight;
  EXPECT_GT(centreX, 0.);
  EXPECT_GT(centreY, 0.);
  EXPECT_LE(std::abs(centreX - 20.), items[0].previewWidth);
  EXPECT_LE(std::abs(centreY - 20.), items[0].previewHeight);
  EXPECT_TRUE(items[1].previewPlaced);
  // A detection which is not in the view at all shows nothing.
  EXPECT_FALSE(items[2].previewPlaced);
}

TEST(ScreenLayout, MoreImportantDetectionShowsItsPreview) {
  // Both detections are at the same place, but the room is only for one preview.
  auto params = LayoutParams();
  params.maxPreviews = 1;
  std::vector<LayoutItem> items {makeItem(500., 400., .4), makeItem(500., 400., .9)};
  ScreenLayout layout {1000., 800., params};
  layout.place(items);

  ASSERT_EQ(1U, layout.previewCount());
  EXPECT_FALSE(items[0].previewPlaced);
  EXPECT_TRUE(items[1].previewPlaced);
}

TEST(ScreenLayout, PreviewWhichIsShownStaysShown) {
  // The user zoomed or another detection arrived; the preview which is already visible must not start jumping around.
  auto params = LayoutParams();
  params.maxPreviews = 1;
  std::vector<LayoutItem> items {makeItem(500., 400., .4), makeItem(500., 400., .9)};
  items[0].wasShown = true;
  items[0].previousIndex = 0;
  ScreenLayout layout {1000., 800., params};
  layout.place(items);

  ASSERT_EQ(1U, layout.previewCount());
  EXPECT_TRUE(items[0].previewPlaced);
  EXPECT_FALSE(items[1].previewPlaced);
}

TEST(ScreenLayout, FreshDetectionOutranksThePreviouslyShownOnes) {
  auto params = LayoutParams();
  params.maxPreviews = 1;
  std::vector<LayoutItem> items {makeItem(500., 400., .4), makeItem(500., 400., .9)};
  items[0].wasShown = true;
  items[1].fresh = true;
  ScreenLayout layout {1000., 800., params};
  layout.place(items);

  ASSERT_EQ(1U, layout.previewCount());
  EXPECT_FALSE(items[0].previewPlaced);
  EXPECT_TRUE(items[1].previewPlaced);
}

TEST(ScreenLayout, HoveredDetectionOutranksEverything) {
  auto params = LayoutParams();
  params.maxPreviews = 1;
  std::vector<LayoutItem> items {makeItem(500., 400., .4), makeItem(500., 400., .9), makeItem(500., 400., .1)};
  items[0].wasShown = true;
  items[1].fresh = true;
  items[2].hovered = true;
  ScreenLayout layout {1000., 800., params};
  layout.place(items);

  ASSERT_EQ(1U, layout.previewCount());
  EXPECT_FALSE(items[0].previewPlaced);
  EXPECT_FALSE(items[1].previewPlaced);
  EXPECT_TRUE(items[2].previewPlaced);
}

TEST(ScreenLayout, ShownPreviewsAreReportedInOrder) {
  auto params = LayoutParams();
  params.maxPreviews = 3;
  std::vector<LayoutItem> items {makeItem(100., 100., .2), makeItem(500., 400., .9), makeItem(700., 600., .5)};
  ScreenLayout layout {1000., 800., params};
  layout.place(items);

  ASSERT_EQ(3U, layout.previewCount());
  // The most important detection is the first one shown, which the next layout uses to keep the order stable.
  EXPECT_EQ(1U, layout.shownPreviews()[0]);
  EXPECT_EQ(2U, layout.shownPreviews()[1]);
  EXPECT_EQ(0U, layout.shownPreviews()[2]);
}

TEST(ScreenLayout, LabelsAreLimitedAndDoNotOverlapEachOther) {
  auto params = LayoutParams();
  params.maxPreviews = 0;
  params.maxLabels = 5;
  std::vector<LayoutItem> items;
  for (size_t i = 0; i < 30; ++i) {
    auto item = makeItem(50. + 20. * static_cast<double>(i % 20), 50. + 60. * static_cast<double>(i / 20), 1.);
    item.labelWidth = 60.;
    item.labelHeight = 16.;
    items.push_back(item);
  }
  ScreenLayout layout {800., 600., params};
  layout.place(items);

  EXPECT_EQ(5U, layout.labelCount());
  size_t shown = 0;
  for (const auto& item : items) {
    shown += item.labelPlaced;
  }
  EXPECT_EQ(5U, shown);
}

TEST(ScreenLayout, LabelOfADetectionIsAtTheTopOfItsPreview) {
  auto params = LayoutParams();
  params.maxPreviews = 1;
  params.maxLabels = 10;
  auto crowded = makeItem(500., 400., .9);
  crowded.labelWidth = 60.;
  crowded.labelHeight = 16.;
  std::vector<LayoutItem> items {crowded};
  ScreenLayout layout {1000., 800., params};
  layout.place(items);

  ASSERT_TRUE(items[0].previewPlaced);
  ASSERT_TRUE(items[0].labelPlaced);
  // The text is a caption of the picture which shows the detection, so it is written to its top and stays a part of it.
  EXPECT_GE(items[0].label.y, items[0].preview.y);
  EXPECT_LE(items[0].label.y + items[0].label.h, items[0].preview.y + items[0].preview.h);
  EXPECT_GE(overlap(items[0].preview, items[0].label), .9 * items[0].label.w * items[0].label.h);
  EXPECT_NEAR(items[0].preview.x + .5 * items[0].preview.w,
    items[0].label.x + .5 * items[0].label.w, 1.);
}

TEST(ScreenLayout, LabelOfADetectionWithoutAPreviewSitsAboveItsMarker) {
  auto params = LayoutParams();
  params.maxPreviews = 0;
  params.maxLabels = 10;
  auto item = makeItem(500., 400., .9);
  item.labelWidth = 60.;
  item.labelHeight = 16.;
  std::vector<LayoutItem> items {item};
  ScreenLayout layout {1000., 800., params};
  layout.place(items);

  ASSERT_TRUE(items[0].labelPlaced);
  // Without a picture to caption, the label is as close to the marker as it can be: right above it.
  EXPECT_LE(items[0].label.y + items[0].label.h, items[0].anchor.y + 1.);
  EXPECT_NEAR(items[0].anchor.x + .5 * items[0].anchor.w,
    items[0].label.x + .5 * items[0].label.w, 1.);
}

TEST(ScreenLayout, NothingIsPlacedCompletelyOutsideOfTheView) {
  std::vector<LayoutItem> items;
  for (size_t i = 0; i < 20; ++i) {
    auto item = makeItem(10. + 30. * static_cast<double>(i), 15., 1.);
    item.labelWidth = 60.;
    item.labelHeight = 16.;
    items.push_back(item);
  }
  ScreenLayout layout {640., 480., LayoutParams()};
  layout.place(items);

  EXPECT_EQ(0U, hiddenAnnotations(items, 640., 480.));
}

TEST(ScreenLayout, ManyDetectionsArePlacedWithoutOverlap) {
  // The layout runs whenever anything changes, so it has to keep its promises even for a large scene.
  auto params = LayoutParams();
  params.maxPreviews = 20;
  params.maxLabels = 200;
  std::vector<LayoutItem> items;
  for (size_t i = 0; i < 1000; ++i) {
    auto item = makeItem(50. + static_cast<double>(i % 50), 50. + static_cast<double>(i / 50), 1.);
    item.labelWidth = 60.;
    item.labelHeight = 16.;
    items.push_back(item);
  }
  ScreenLayout layout {1600., 1000., params};
  layout.place(items);

  EXPECT_LE(layout.previewCount(), params.maxPreviews);
  EXPECT_LE(layout.labelCount(), params.maxLabels);
  EXPECT_EQ(0U, overlappingPreviews(items));
  EXPECT_EQ(0U, hiddenAnnotations(items, 1600., 1000.));
}

TEST(ScreenLayout, LabelsOfCrowdedDetectionsAreStillShown) {
  // A dense cluster of detections fills the whole place with its markers, but the labels have to find room anyway - a
  // label over a few markers is still readable, while no label at all is not what the user asked for.
  auto params = LayoutParams();
  params.maxPreviews = 0;
  params.maxLabels = 100;
  auto items = makeCrowd(30);
  for (auto& item : items) {
    item.previewWidth = 0.;
    item.previewHeight = 0.;
    item.labelWidth = 60.;
    item.labelHeight = 16.;
  }
  ScreenLayout layout {800., 600., params};
  layout.place(items);

  // In a pile this narrow there is room for only a few labels of the most important detections, but there must be
  // some, and they must not overlap each other.
  EXPECT_GE(layout.labelCount(), 5U);
  EXPECT_EQ(0U, overlappingLabels(items));
  EXPECT_EQ(0U, hiddenAnnotations(items, 800., 600.));
}

TEST(ScreenLayout, LabelDoesNotOverlapAPreviewOfAnotherDetection) {
  auto params = LayoutParams();
  params.maxPreviews = 1;
  params.maxLabels = 10;
  auto previewed = makeItem(300., 300., .9);
  auto crowded = makeItem(320., 300., .8);
  crowded.previewWidth = 0.;
  crowded.previewHeight = 0.;
  crowded.labelWidth = 200.;
  crowded.labelHeight = 16.;
  std::vector<LayoutItem> items {previewed, crowded};
  ScreenLayout layout {800., 600., params};
  layout.place(items);

  ASSERT_TRUE(items[0].previewPlaced);
  ASSERT_TRUE(items[1].labelPlaced);
  EXPECT_FLOAT_EQ(0.f, overlap(items[0].preview, items[1].label));
}

TEST(LayoutItem, PreviewRankPrefersHoveredThenFreshThenShown) {
  cras_rviz_vision_msgs::LayoutItem base;
  base.priority = 1.;
  auto shown = base;
  shown.wasShown = true;
  shown.previousIndex = 3;
  auto later = shown;
  later.previousIndex = 7;
  auto fresh = base;
  fresh.fresh = true;
  auto hovered = base;
  hovered.hovered = true;
  auto selected = base;
  selected.selected = true;
  auto important = base;
  important.priority = 1e9;

  EXPECT_GT(shown.previewRank(), base.previewRank());
  // The previews which were shown first keep their place against the ones shown later.
  EXPECT_GT(shown.previewRank(), later.previewRank());
  EXPECT_GT(fresh.previewRank(), shown.previewRank());
  EXPECT_GT(hovered.previewRank(), fresh.previewRank());
  // Only an unreasonably important detection can take the preview from the hovered one.
  EXPECT_GT(important.previewRank(), hovered.previewRank());
  // The detection the user clicked on has nothing to lose to, not even to an unreasonably important one.
  EXPECT_GT(selected.previewRank(), hovered.previewRank());
  EXPECT_GT(selected.previewRank(), important.previewRank());
}

TEST(ScreenLayout, SelectedDetectionTakesThePlaceOfAShownOne) {
  auto params = LayoutParams();
  params.maxPreviews = 1;
  std::vector<LayoutItem> items {makeItem(500., 400., .9), makeItem(500., 300., .4)};
  items[0].wasShown = true;
  items[1].selected = true;
  ScreenLayout layout {1000., 800., params};
  layout.place(items);

  // The user asked to see the second detection, so it takes the place over no matter how well the first one was placed.
  ASSERT_EQ(1U, layout.previewCount());
  EXPECT_FALSE(items[0].previewPlaced);
  EXPECT_TRUE(items[1].previewPlaced);
}

TEST(ScreenLayout, SelectedPreviewIsShownEvenInAFullView) {
  // A small view full of detections: there is not a free place for every picture, and the layout has to choose.
  auto params = LayoutParams();
  params.maxPreviews = 100;
  auto crowded = makeCrowd(12, 150., 120.);
  ScreenLayout without {320., 240., params};
  without.place(crowded);
  size_t leftOut = 0;
  for (size_t i = 0; i < crowded.size(); ++i) {
    if (!crowded[i].previewPlaced) {
      leftOut = i;
      break;
    }
  }
  ASSERT_FALSE(crowded[leftOut].previewPlaced);  // the point of the test is a detection which gets nothing

  auto selected = crowded;
  selected[leftOut].selected = true;
  ScreenLayout layout {320., 240., params};
  layout.place(selected);

  // The detection the user asked for gets its picture shown over the ones in its way; the others keep their promise
  // of not overlapping each other.
  EXPECT_TRUE(selected[leftOut].previewPlaced);
  EXPECT_GE(layout.previewCount(), without.previewCount());
}

TEST(EstimateLabelWidth, GrowsWithTheFontAndTheText) {
  EXPECT_GT(estimateLabelWidth(10, 20), estimateLabelWidth(10, 14));
  EXPECT_GT(estimateLabelWidth(20, 14), estimateLabelWidth(10, 14));
  EXPECT_GT(estimateLabelWidth(10, 14), 0.);
}
