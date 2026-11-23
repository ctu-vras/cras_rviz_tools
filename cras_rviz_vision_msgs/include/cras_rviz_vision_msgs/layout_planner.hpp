// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Placement of the image previews and text labels of the detections in the RViz view.
 *
 * When several detections are close to each other (which is common when looking at the scene from the top down),
 * drawing all their annotations would result in an unreadable pile of overlapping rectangles. The layout places the
 * most important annotations first to the freest place near their detection and leaves the rest of the detections
 * marked by their plain marker only. The previews have a limited size in the view, whatever the shape of the picture
 * they show, so zooming in always makes room for more of them.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <utility>
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
  double area() const {
    return w > 0. && h > 0. ? w * h : 0.;
  }
};

/**
 * \brief One detection to place its annotations in the view.
 *
 * The marker of the detection is always drawn at its position and is always reserved for it; the image preview and the
 * text label are the annotations the layout looks for room.
 */
struct LayoutItem {
  ScreenRect anchor;  //!< Rectangle of the marker of the detection.
  double priority {0.};  //!< The higher the number, the more important the detection (below all the bonuses below).
  bool fresh {false};  //!< The detection just arrived; its preview outranks the previews shown before.
  bool wasShown {false};  //!< The preview of this detection was shown by the previous layout.
  int previousIndex {0};  //!< Order of the preview in the previous layout (0 for the first one).
  bool hovered {false};  //!< The user points at this detection; its preview has absolute priority.
  bool selected {false};  //!< The user asked to highlight this detection; its preview has the highest priority.
  double previewWidth {0.};  //!< Width of the image preview [px]; zero width means the detection has no preview.
  //! Height of the image preview [px]. The picture keeps the shape of the part of the camera image it shows, so both
  //! dimensions follow from its aspect ratio and from the box the previews are limited to (see fitIntoBox).
  double previewHeight {0.};
  double stem {0.};  //!< Width of the line connecting the preview to its marker [px]; it has to stay visible.
  double labelWidth {0.};  //!< Size of the text label [px]; zero width means the detection has no label.
  double labelHeight {0.};

  // The results of the layout.
  bool previewPlaced {false};  //!< Whether there was room for the image preview.
  ScreenRect preview;  //!< Place of the image preview.
  bool stemPlaced {false};  //!< Whether the line to the marker also fits without covering anything.
  bool labelPlaced {false};  //!< Whether there was room for the text label.
  ScreenRect label;  //!< Place of the text label.

  //! How important it is to show the preview of this detection (see ScreenLayout for the meaning of the bonuses).
  double previewRank() const {
    auto value = priority;
    if (wasShown) {
      value += 1e6 - static_cast<double>(previousIndex);
    }
    if (fresh) {
      value += 2e6;
    }
    if (hovered) {
      value += 3e6;
    }
    if (selected) {
      // The detection the user asked for (by clicking it, in the view or in another image) outranks everything else:
      // it is also drawn with a frame of its own and opaque, so hiding it somewhere out of the way would be a waste.
      // The priority of a detection is at most a few units, so nothing but a click can take its preview away.
      value += 1e9;
    }
    return value;
  }
};

/**
 * \brief Rough width of a text label with the given number of characters.
 *
 * The exact width depends on the font and is known only to it, but the layout only needs to know how much space to
 * keep for the label, which this estimate is good enough for.
 *
 * \param[in] characterCount Number of characters of the label.
 * \param[in] fontHeight Height of the font (in pixels).
 * \return The estimated width of the label (in pixels).
 */
double estimateLabelWidth(size_t characterCount, double fontHeight);

/**
 * \brief The size of a picture of the given aspect ratio which fits into a box of the given size.
 *
 * The picture keeps its shape (it is never stretched), and neither of its dimensions grows over the box it belongs to.
 * A wide picture is therefore limited by the width of the box and a tall one (the cutout of a person standing) by its
 * height - which is what keeps a tall detection from needing a preview many times taller than the whole view.
 *
 * \param[in] boxWidth Width of the box the picture has to fit into [px].
 * \param[in] boxHeight Height of the box the picture has to fit into [px]. A non-positive height makes it a square box.
 * \param[in] aspectRatio Width of the picture divided by its height. A non-positive value is taken for a square one.
 * \return The width and the height of the picture [px].
 */
std::pair<double, double> fitIntoBox(double boxWidth, double boxHeight, double aspectRatio);

//! Tuning parameters of the layout.
struct LayoutParams {
  double margin {6.};  //!< Minimal spacing between two neighbouring annotations [px].
  double minCoverage {.5};  //!< Minimal fraction of the annotation that has to be visible in the view.
  size_t maxPreviews {10};  //!< Maximal number of image previews to show at once.
  size_t maxLabels {200};  //!< Maximal number of text labels to show at once.
  int cell {16};  //!< Size of one cell of the occupancy grid the layout works with [px].
};

/**
 * \brief Placement of the annotations of the detections in one view.
 *
 * The layout keeps an occupancy grid over the view and fills it first with the markers of all detections. It then
 * places the image previews in the order of decreasing \ref LayoutItem::previewRank "preview rank" and the text labels
 * in the same order after them, always to the first place around the detection which is free and inside the view.
 */
class ScreenLayout {
public:
  ScreenLayout(double viewWidth, double viewHeight, const LayoutParams& params);

  /**
   * \brief Reserves the markers and places the previews and the labels of all detections.
   * \param[in,out] items The detections to place; the layout is stored in them.
   */
  void place(std::vector<LayoutItem>& items);

  /**
   * \brief Indices of the detections whose preview is shown, in the order they were placed.
   *
   * The display passes it to the next layout as \ref LayoutItem::previousIndex, so that the previews which are already
   * shown stay shown when the layout changes.
   */
  const std::vector<size_t>& shownPreviews() const;

  //! Number of placed image previews.
  size_t previewCount() const;
  //! Number of placed text labels.
  size_t labelCount() const;

protected:
  //! Whether the rectangle is visible in the view (enough of it to be worth showing).
  bool inView(const ScreenRect& rect) const;
  /**
   * \brief Whether the rectangle fits into the view and does not overlap any preview or label but its own.
   *
   * The markers of the detections are not taken into account: they are small, always drawn, and in a dense scene they
   * fill the whole view, so respecting them would make it impossible to place any text at all.
   */
  bool free(const ScreenRect& rect, int owner, const ScreenRect* ignore = nullptr) const;
  /**
   * \brief Marks the cells of the rectangle as occupied by the given annotation.
   * \param[in] blocks Whether the annotation keeps other annotations away (previews and labels do, markers do not).
   */
  void reserve(const ScreenRect& rect, int owner, bool blocks = true);
  //! Rectangle of an annotation enlarged by the spacing, so that its neighbours keep their distance from it.
  ScreenRect spaced(const ScreenRect& rect) const;
  /**
   * \brief Looks for a free place for an annotation, starting from the place it would ideally have.
   * \param[in] anchor Place the annotation belongs to (the marker of the detection); the line to it has to stay free.
   * \param[in] natural The place the annotation would ideally have; its size is the size of the annotation.
   * \param[in] maxShift How far the annotation may get from its natural place to avoid the other ones [px].
   * \param[in] stem Width of the line to the anchor which has to stay free; 0 does not look at it.
   * \param[in] owner Index of the detection the annotation belongs to.
   * \param[out] result Place found for the annotation.
   * \return Whether a free place was found.
   */
  bool findPlace(const ScreenRect& anchor, const ScreenRect& natural, double maxShift, double stem, int owner,
      ScreenRect& result);
  /**
   * \brief Looks for a place which is at least mostly visible, no matter whether something is shown there already.
   *
   * The preview of the detection the user highlighted is shown over the annotations in its way rather than not shown at
   * all - the click is the strongest claim on the view there is. Only a preview which would be nothing but a fragment
   * outside of the view is given up; nobody could recognise the detection in it.
   *
   * \param[in] natural The place the preview would ideally have; its size is the size of the preview.
   * \param[in] maxShift How far the preview may get from its natural place [px].
   * \param[out] result Place found for the preview.
   * \return Whether a place mostly inside the view was found.
   */
  bool findVisiblePlace(const ScreenRect& natural, double maxShift, ScreenRect& result) const;
  /**
   * \brief Rectangle covering the line between the marker and the placed annotation.
   *
   * It is the place the line connecting a preview with its marker needs; it is empty when the marker already is at the
   * preview (the preview is then obviously the preview of that detection and no line is needed).
   */
  static ScreenRect stemRect(const ScreenRect& anchor, const ScreenRect& place, double thickness);
  //! Offsets (in units of the search step) which are tried around the natural place, nearest first.
  static const std::vector<std::pair<int, int>>& searchPattern();

  double viewWidth_;  //!< Width of the view [px].
  double viewHeight_;  //!< Height of the view [px].
  LayoutParams params_;  //!< Tuning parameters.
  int columns_ {0};  //!< Number of cells in one row of the occupancy grid.
  int rows_ {0};  //!< Number of cells in one column of the occupancy grid.
  std::vector<int> occupied_;  //!< Index of the annotation which occupies each cell, -1 for a free cell.
  std::vector<uint8_t> blocking_;  //!< Whether each cell keeps the other annotations away.
  std::vector<size_t> shownPreviews_;  //!< Indices of the detections whose preview was placed.
  size_t labelCount_ {0};  //!< Number of labels placed so far.
};

}  // namespace cras_rviz_vision_msgs
