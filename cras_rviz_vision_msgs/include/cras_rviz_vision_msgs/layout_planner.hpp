// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Placement of 2D annotations (image previews and text labels) in the RViz view.
 *
 * When several detections are close to each other (which is common when looking at the scene from the top down),
 * naive drawing of all annotations would result in an unreadable pile of overlapping rectangles. The layout planner
 * tries to place every annotation to a free place near its detection and decides which detections are worth showing
 * with an image preview and which are only plotted as a plain marker.
 */

#pragma once

#include <algorithm>
#include <cstddef>
#include <vector>

namespace cras_rviz_vision_msgs {

//! A rectangle in the 2D screen space (pixels, the y-axis pointing down).
struct ScreenRect {
  double x {0};  //!< Horizontal position of the left edge.
  double y {0};  //!< Vertical position of the top edge.
  double w {0};  //!< Width.
  double h {0};  //!< Height.

  ScreenRect() = default;
  ScreenRect(double x_, double y_, double w_, double h_)
      : x(x_), y(y_), w(w_), h(h_) {}

  //! Area of the rectangle.
  double area() const{
    return w > 0. && h > 0. ? w * h : 0.;
  }

  //! Position of the rectangle centre.
  void centre(double& cx, double& cy) const{
    cx = x + w / 2.;
    cy = y + h / 2.;
  }
};

//! Intersection of two rectangles (with a negative size when they do not overlap).
inline ScreenRect intersect(const ScreenRect& a, const ScreenRect& b) {
  const double x = std::max(a.x, b.x);
  const double y = std::max(a.y, b.y);
  return {x, y, std::min(a.x + a.w, b.x + b.w) - x, std::min(a.y + a.h, b.y + b.h) - y};
}

//! Relative position of an annotation w.r.t. the detection it belongs to.
enum class AnchorSide {
  CENTER,  //!< The annotation is centred at the detection position.
  RIGHT,  //!< The annotation is to the right of the detection.
  LEFT,  //!< The annotation is to the left of the detection.
  TOP,  //!< The annotation is above the detection.
  BOTTOM,  //!< The annotation is below the detection.
};

//! Rectangle of the given size placed around the given anchor in the given direction.
inline ScreenRect anchoredRect(const ScreenRect& anchor, AnchorSide side, double width, double height,
    double margin) {
  switch (side) {
    case AnchorSide::RIGHT:
      return {anchor.x + anchor.w + margin, anchor.y + (anchor.h - height) / 2., width, height};
    case AnchorSide::LEFT:
      return {anchor.x - margin - width, anchor.y + (anchor.h - height) / 2., width, height};
    case AnchorSide::TOP:
      return {anchor.x + (anchor.w - width) / 2., anchor.y - margin - height, width, height};
    case AnchorSide::BOTTOM:
      return {anchor.x + (anchor.w - width) / 2., anchor.y + anchor.h + margin, width, height};
    case AnchorSide::CENTER:
    default:
      return {anchor.x + (anchor.w - width) / 2., anchor.y + (anchor.h - height) / 2., width, height};
  }
}

//! One annotation to be placed.
struct LayoutItem {
  ScreenRect anchor;  //!< Rectangle of the marker drawn directly at the detection position.
  double priority {0.};  //!< The higher the number, the more important the detection.
  double areaWeight {1.};  //!< Rough screen area of the detection marker (used to break priority ties).
  double maxWidth {100.};  //!< Maximal width of the image preview in pixels.
  double aspectRatio {1.};  //!< Width / height of the image preview.
  double extraHeight {0.};  //!< Height of the label belonging to the item (px), 0 if there is no label.
  double labelWidth {0.};  //!< Width of the label (px), see estimateLabelWidth(). Defaults to maxWidth.
  bool hasPreview {false};  //!< Whether an image preview is available for this detection.
  int id {-1};  //!< Identifier of the item, copied to the resulting PlacedItem.
};

/**
 * \brief Rough width of a text label with the given number of characters.
 *
 * The exact width depends on the font and is known only to it, but the placement only needs to know how much space to
 * keep for the label, which this estimate is good enough for.
 *
 * \param[in] characterCount Number of characters of the label.
 * \param[in] fontHeight Height of the font (in pixels).
 * \return The estimated width of the label (in pixels).
 */
double estimateLabelWidth(size_t characterCount, double fontHeight);

//! One placed annotation.
struct PlacedItem {
  int id {-1};  //!< Identifier of the corresponding LayoutItem.
  bool preview {false};  //!< Whether the image preview fits into the view.
  AnchorSide anchor {AnchorSide::CENTER};  //!< Where the annotation was placed.
  ScreenRect rect;  //!< Rectangle of the placed annotation (in pixels).
  ScreenRect labelRect;  //!< Place found for the label of the detection (empty when nothing was free).
};

//! Placement of annotations in the 2D view.
class LayoutPlanner {
public:
  //! Tuning parameters of the placement.
  struct Params {
    double margin {6.0};  //!< Minimal spacing between two neighbouring annotations (in pixels).
    double minCoverage {0.5};  //!< Minimal fraction of the annotation that must be visible in the view.
    double minPreviewWidth {24.0};  //!< Do not show a preview smaller than this (in pixels).
    int previewScales {4};  //!< Number of progressively smaller previews tried to find a free place.
    double maxLabelOverlap {.35};  //!< How much (fraction of its area) can a label share with other annotations.
    //! Maximal number of image previews to place (the most important detections get one), 0 for no limit. Showing a
    //! preview means keeping a texture in memory, so an unlimited number of previews would be needlessly expensive.
    size_t maxPreviews {0};
  };

  /**
   * \brief Places the given annotations into the view.
   *
   * The items are processed in the order of decreasing priority. Each item is placed to the first free place that is
   * found, starting from the most natural position (the image preview is centred at the detection, text annotations
   * are placed to the right of it). If no image preview fits (even after shrinking it), the detection is only marked
   * by its plain marker.
   *
   * \param[in,out] items Items to place.
   * \param[in] viewWidth Width of the view in pixels.
   * \param[in] viewHeight Height of the view in pixels.
   * \return The placed annotations (only for the items that were placed).
   */
  static std::vector<PlacedItem> plan(std::vector<LayoutItem> items, double viewWidth, double viewHeight);

  /**
   * \brief Places the given annotations into the view.
   * \param[in,out] items Items to place.
   * \param[in] viewWidth Width of the view in pixels.
   * \param[in] viewHeight Height of the view in pixels.
   * \param[in] params Placement parameters.
   * \return The placed annotations (only for the items that were placed).
   */
  static std::vector<PlacedItem> plan(std::vector<LayoutItem> items, double viewWidth, double viewHeight,
      const Params& params);
};

}  // namespace cras_rviz_vision_msgs
