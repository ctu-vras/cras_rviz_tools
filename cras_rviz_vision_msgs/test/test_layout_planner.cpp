// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Tests of the annotation placement.
 * \author Czech Technical University in Prague
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <vector>

#include "cras_rviz_vision_msgs/layout_planner.hpp"

using cras_rviz_vision_msgs::AnchorSide;
using cras_rviz_vision_msgs::LayoutItem;
using cras_rviz_vision_msgs::LayoutPlanner;
using cras_rviz_vision_msgs::PlacedItem;
using cras_rviz_vision_msgs::ScreenRect;

namespace {

//! Item with sensible defaults, anchored to the given rectangle.
LayoutItem makeItem(int id, double x, double y, double width, double height, double priority = 1.) {
  LayoutItem item;
  item.id = id;
  item.anchor = {x, y, width, height};
  item.priority = priority;
  item.areaWeight = width * height;
  item.maxWidth = 100.;
  item.aspectRatio = 1.;
  item.hasPreview = true;
  return item;
}

//! Number of items that were placed with an image preview.
size_t previewCount(const std::vector<PlacedItem>& placed) {
  return std::count_if(placed.begin(), placed.end(), [](const PlacedItem& item) {return item.preview;});
}

//! The placement of the given item, or nullptr if it is not there.
const PlacedItem* find(const std::vector<PlacedItem>& placed, int id) {
  const auto item = std::find_if(placed.begin(), placed.end(),
      [id](const PlacedItem& placedItem) {return placedItem.id == id;});
  return item == placed.end() ? nullptr : &*item;
}

//! Checks that no two placed previews overlap each other.
void expectNoOverlaps(const std::vector<PlacedItem>& placed) {
  for (const auto& first : placed) {
    if (!first.preview) {
      continue;
    }
    for (const auto& second : placed) {
      if (!second.preview || first.id >= second.id) {
        continue;
      }
      EXPECT_LE(intersect(first.rect, second.rect).area(), 1e-6) <<
        "Previews " << first.id << " and " << second.id << " overlap.";
    }
  }
}

}  // namespace

TEST(LayoutPlanner, SingleItemGetsPreview)
{
  const auto placed = LayoutPlanner::plan({makeItem(0, 200, 200, 60, 60)}, 800, 600);

  ASSERT_EQ(placed.size(), 1);
  EXPECT_TRUE(placed[0].preview);
  EXPECT_EQ(placed[0].id, 0);
  // The preview has the requested size and is centred at the detection.
  EXPECT_DOUBLE_EQ(placed[0].rect.w, 100.);
  EXPECT_DOUBLE_EQ(placed[0].rect.h, 100.);
  EXPECT_NEAR(placed[0].rect.x + 50., 230., 1.);
  EXPECT_NEAR(placed[0].rect.y + 50., 230., 1.);
}

TEST(LayoutPlanner, DetectionsWithoutImageAreOnlyMarked)
{
  auto item = makeItem(0, 200, 200, 60, 60);
  item.hasPreview = false;

  const auto placed = LayoutPlanner::plan({item}, 800, 600);

  ASSERT_EQ(placed.size(), 1);
  EXPECT_FALSE(placed[0].preview);
  EXPECT_DOUBLE_EQ(placed[0].rect.x, item.anchor.x);
  EXPECT_DOUBLE_EQ(placed[0].rect.w, item.anchor.w);
}

TEST(LayoutPlanner, DistantDetectionsGetTheirOwnSpace)
{
  const std::vector<LayoutItem> items = {
    makeItem(0, 20, 20, 40, 40), makeItem(1, 400, 300, 40, 40), makeItem(2, 700, 520, 40, 40)};

  const auto placed = LayoutPlanner::plan(items, 800, 600);

  EXPECT_EQ(previewCount(placed), 3);
  expectNoOverlaps(placed);
}

TEST(LayoutPlanner, OverlappingDetectionsShareTheSpace)
{
  // Three detections on exactly the same place. Only the space around them is available for the annotations, so not
  // all of them can get a preview.
  const std::vector<LayoutItem> items = {
    makeItem(0, 400, 300, 20, 20, 3.), makeItem(1, 400, 300, 20, 20, 2.), makeItem(2, 400, 300, 20, 20, 1.)};

  const auto placed = LayoutPlanner::plan(items, 800, 600);

  ASSERT_EQ(placed.size(), 3);
  const auto previews = previewCount(placed);
  EXPECT_GT(previews, 0);
  EXPECT_LT(previews, 3);
  expectNoOverlaps(placed);

  // The most probable detection must always win the fight about the best place.
  const auto* best = find(placed, 0);
  ASSERT_NE(best, nullptr);
  EXPECT_TRUE(best->preview);
  EXPECT_EQ(best->anchor, AnchorSide::CENTER);
}

TEST(LayoutPlanner, MoreSpaceMeansMorePreviews)
{
  // Once seen from far away (the detections and their previews are large w.r.t. the free space) and once after
  // zooming in (the detections are further apart from each other and each preview needs less space).
  std::vector<LayoutItem> items;
  for (int i = 0; i < 12; ++i) {
    items.push_back(makeItem(i, 380 + 6 * (i % 4), 280 + 6 * (i / 4), 20, 20));
  }
  const auto far = LayoutPlanner::plan(items, 800, 600);

  items.clear();
  for (int i = 0; i < 12; ++i) {
    auto item = makeItem(i, 200 + 100 * (i % 4), 150 + 100 * (i / 4), 20, 20);
    item.maxWidth = 30;
    items.push_back(item);
  }
  const auto close = LayoutPlanner::plan(items, 800, 600);

  EXPECT_GT(previewCount(close), previewCount(far));
  EXPECT_EQ(previewCount(close), 12);
  expectNoOverlaps(far);
  expectNoOverlaps(close);
}

TEST(LayoutPlanner, PlacesLabelsOfCrowdedDetections)
{
  // Two detections close to each other, both with a label but without an image. Their labels must not overlap each
  // other, otherwise a crowd of detections becomes unreadable.
  auto first = makeItem(0, 300, 300, 10, 10, 2.);
  first.extraHeight = 20.;
  first.labelWidth = 80.;
  first.hasPreview = false;
  auto second = makeItem(1, 302, 301, 10, 10, 1.);
  second.extraHeight = 20.;
  second.labelWidth = 80.;
  second.hasPreview = false;

  const auto placed = LayoutPlanner::plan({first, second}, 800, 600);

  ASSERT_EQ(placed.size(), 2);
  EXPECT_EQ(previewCount(placed), 0);
  ASSERT_GT(placed[0].labelRect.area(), 0.);
  ASSERT_GT(placed[1].labelRect.area(), 0.);
  EXPECT_EQ(intersect(placed[0].labelRect, placed[1].labelRect).area(), 0.);
}

TEST(LayoutPlanner, EstimatesWidthOfLongerLabels)
{
  // The labels are placed as rectangles, so the planner needs to know how wide the text is. The exact width is known
  // only to the font, but a longer label must certainly need more space.
  EXPECT_GT(cras_rviz_vision_msgs::estimateLabelWidth(20, 14),
    cras_rviz_vision_msgs::estimateLabelWidth(10, 14));
  EXPECT_GT(cras_rviz_vision_msgs::estimateLabelWidth(10, 14), 0.);
}

TEST(LayoutPlanner, PlacesLabelsOfDifferentWidths)
{
  // Detections with labels of different lengths in a free view. Two labels must never share the same place, so a label
  // which would overlap a longer one placed near a more important detection is moved elsewhere.
  std::vector<LayoutItem> items;
  for (int i = 0; i < 5; ++i) {
    auto item = makeItem(i, 100 + 130 * i, 300, 10, 10, 2. - .1 * i);
    item.hasPreview = false;
    item.extraHeight = 20.;
    item.labelWidth = cras_rviz_vision_msgs::estimateLabelWidth(4 + i, 20.);
    items.push_back(item);
  }

  const auto placed = LayoutPlanner::plan(items, 800, 600);

  ASSERT_EQ(placed.size(), items.size());
  for (size_t i = 0; i < placed.size(); ++i) {
    for (size_t j = i + 1; j < placed.size(); ++j) {
      EXPECT_EQ(intersect(placed[i].labelRect, placed[j].labelRect).area(), 0.) <<
        "Labels " << i << " and " << j << " overlap.";
    }
    EXPECT_GT(placed[i].labelRect.area(), 0.) << "Label " << i << " was left out although there is enough space.";
  }
}

TEST(LayoutPlanner, LabelsOfDenseCrowdDoNotOverlap)
{
  // The detections are so close to each other that not all their labels fit. Those which are placed must still be
  // readable, i.e. must not overlap each other.
  std::vector<LayoutItem> items;
  for (int i = 0; i < 30; ++i) {
    auto item = makeItem(i, 100 + 12 * i, 300, 8., 8., 2. - .05 * i);
    item.hasPreview = false;
    item.extraHeight = 20.;
    item.labelWidth = cras_rviz_vision_msgs::estimateLabelWidth(10, 20.);
    items.push_back(item);
  }

  const auto placed = LayoutPlanner::plan(items, 800, 600);

  ASSERT_EQ(placed.size(), items.size());
  size_t labelled = 0;
  for (size_t i = 0; i < placed.size(); ++i) {
    for (size_t j = i + 1; j < placed.size(); ++j) {
      EXPECT_EQ(intersect(placed[i].labelRect, placed[j].labelRect).area(), 0.);
    }
    labelled += placed[i].labelRect.area() > 0.;
  }
  EXPECT_GT(labelled, 0U);  // the free places are used
  // The detections are 12 px apart and the labels are 120 px wide, so they cannot all fit into the free places around
  // their detections - and the ones which do not fit must not be drawn over the other texts.
  EXPECT_LT(labelled, items.size());
}

TEST(LayoutPlanner, LeavesOutLabelWithoutFreePlace)
{
  // A detection in a corner of a tiny view has nowhere to put its label. Rather than drawing it over everything else,
  // the planner reports no place for it and the display does not draw it.
  auto item = makeItem(0, 0, 0, 10, 10, 1.);
  item.extraHeight = 20.;
  item.labelWidth = 80.;

  LayoutPlanner::Params params;
  params.maxLabelOverlap = 0.;  // the label may not share its place with anything else

  const auto placed = LayoutPlanner::plan({item}, 25, 25, params);

  ASSERT_EQ(placed.size(), 1);
  EXPECT_EQ(placed[0].labelRect.area(), 0.);
}

TEST(LayoutPlanner, AnnotationsStayInsideTheView)
{
  const std::vector<LayoutItem> items = {
    makeItem(0, 0, 0, 20, 20), makeItem(1, 780, 580, 20, 20), makeItem(2, 790, 10, 20, 20)};

  const auto placed = LayoutPlanner::plan(items, 800, 600);

  ASSERT_EQ(placed.size(), 3);
  for (const auto& item : placed) {
    if (!item.preview) {
      continue;
    }
    const auto visible = intersect(item.rect, ScreenRect{0., 0., 800., 600.}).area();
    EXPECT_GE(visible / item.rect.area(), .5) << "Preview " << item.id << " is mostly outside of the view.";
  }
  expectNoOverlaps(placed);
}

TEST(LayoutPlanner, ExtraSpaceIsKeptForLabels)
{
  auto item = makeItem(0, 400, 300, 20, 20, 2.);
  item.extraHeight = 30.;

  const auto placed = LayoutPlanner::plan({item}, 800, 600);

  ASSERT_EQ(placed.size(), 1);
  EXPECT_TRUE(placed[0].preview);
  // The preview itself is still only as big as the shown image is.
  EXPECT_DOUBLE_EQ(placed[0].rect.h, placed[0].rect.w);
}

TEST(LayoutPlanner, EmptyInputIsHandled)
{
  const auto placed = LayoutPlanner::plan({}, 800, 600);
  EXPECT_TRUE(placed.empty());
}

TEST(LayoutPlanner, ZeroSizedViewIsHandled)
{
  const auto placed = LayoutPlanner::plan({makeItem(0, 0, 0, 20, 20)}, 0, 0);
  ASSERT_EQ(placed.size(), 1);
  EXPECT_FALSE(placed[0].preview);
}

TEST(LayoutPlanner, LimitsTheNumberOfPreviews)
{
  const std::vector<LayoutItem> items = {
    makeItem(0, 20, 20, 40, 40, 3.), makeItem(1, 400, 300, 40, 40, 2.), makeItem(2, 700, 520, 40, 40, 1.)};

  LayoutPlanner::Params params;
  params.maxPreviews = 2;  // there would be enough space for all the previews

  const auto placed = LayoutPlanner::plan(items, 800, 600, params);

  ASSERT_EQ(placed.size(), 3);
  EXPECT_EQ(previewCount(placed), 2);
  // The most probable detections are the ones which get the limited number of previews.
  EXPECT_TRUE(find(placed, 0)->preview);
  EXPECT_TRUE(find(placed, 1)->preview);
  EXPECT_FALSE(find(placed, 2)->preview);
  expectNoOverlaps(placed);
}

TEST(LayoutPlanner, PlacesManyDetections)
{
  // A scene with a thousand detections must still be placeable in a small fraction of the frame time.
  std::vector<LayoutItem> items;
  for (int i = 0; i < 1000; ++i) {
    items.push_back(makeItem(i, 20. * (i % 40), 20. * (i / 40), 12., 12., (i % 7) / 7.));
  }

  LayoutPlanner::Params params;
  params.maxPreviews = 100;

  const auto start = std::chrono::steady_clock::now();
  const auto placed = LayoutPlanner::plan(items, 1920, 1080, params);
  const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
    start);

  ASSERT_EQ(placed.size(), 1000U);
  EXPECT_GT(previewCount(placed), 0U);
  EXPECT_LE(previewCount(placed), 100U);
  expectNoOverlaps(placed);
  EXPECT_LT(elapsed.count(), 400) << "Placing 1000 detections took " << elapsed.count() << " ms.";
}

TEST(LayoutPlanner, PlacementIsDeterministic)
{
  std::vector<LayoutItem> items;
  for (int i = 0; i < 20; ++i) {
    items.push_back(makeItem(i, 300 + i, 250 + i % 3, 30, 30, i % 2 ? 1.5 : 1.));
  }

  const auto first = LayoutPlanner::plan(items, 640, 480);
  const auto second = LayoutPlanner::plan(items, 640, 480);

  ASSERT_EQ(first.size(), second.size());
  for (size_t i = 0; i < first.size(); ++i) {
    EXPECT_EQ(first[i].id, second[i].id);
    EXPECT_EQ(first[i].preview, second[i].preview);
    EXPECT_DOUBLE_EQ(first[i].rect.x, second[i].rect.x);
    EXPECT_DOUBLE_EQ(first[i].rect.y, second[i].rect.y);
  }
}
