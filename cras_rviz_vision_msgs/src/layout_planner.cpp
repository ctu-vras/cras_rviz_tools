// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Implementation of the annotation placement.
 * \author Czech Technical University in Prague
 */

#include "cras_rviz_vision_msgs/layout_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <utility>

namespace cras_rviz_vision_msgs {

namespace {

using Sides = std::array<AnchorSide, 5>;

//! The order in which the placement candidates are tried. The preview belongs to the detection, so it is drawn right
//! at its position; if that place is taken by a more important detection, the preview moves next to the detection and
//! a plain marker shows the detection itself.
constexpr Sides IMAGE_SIDES = {
  AnchorSide::CENTER, AnchorSide::RIGHT, AnchorSide::BOTTOM, AnchorSide::LEFT, AnchorSide::TOP};

//! The order in which the placement candidates are tried - a marker should not hide the detection itself.
constexpr Sides MARKER_SIDES = {
  AnchorSide::RIGHT, AnchorSide::LEFT, AnchorSide::BOTTOM, AnchorSide::TOP, AnchorSide::CENTER};

//! How wide a single character of the label text is, as a fraction of the font height.
constexpr double LABEL_WIDTH_FACTOR = .6;

//! Rectangle with the given margin added to all its sides.
inline ScreenRect expanded(const ScreenRect& rect, double margin) {
  return {rect.x - margin, rect.y - margin, rect.w + 2. * margin, rect.h + 2. * margin};
}

/**
 * \brief Rectangles which are already taken, indexed by their position in the view.
 *
 * The placement needs to know which rectangles overlap a candidate place, and how many other detections the candidate
 * place covers. Doing that naively (by scanning all the already placed rectangles) would make the placement quadratic
 * in the number of detections, which is too slow for scenes with hundreds of them. The grid keeps the number of
 * rectangles actually examined small.
 */
class RectIndex {
public:
  /**
   * \param[in] cellSize Size of one grid cell [px]. It should be comparable to the size of the queried rectangles.
   */
  explicit RectIndex(double cellSize)
      : cellSize_(std::max(1., cellSize)) {}

  //! Adds the given rectangle to the index.
  void insert(const ScreenRect& rect) {
    if (!(rect.w > 0.) || !(rect.h > 0.)) {
      return;  // an empty rectangle never overlaps anything
    }
    const auto index = static_cast<uint32_t>(rects_.size());
    rects_.push_back(rect);
    if (!collectCells(rect) || scratch_.size() > MAX_CELLS_PER_RECT) {
      oversized_.push_back(index);  // such a large rectangle has to be examined in every query anyway
      return;
    }
    for (const auto& cell : scratch_) {
      cells_[cell].push_back(index);
    }
  }

  /**
   * \brief Calls the given callback for every stored rectangle which can overlap the given one.
   *
   * Each rectangle is passed at most once, so the callback can e.g. sum the overlap areas without counting the same
   * rectangle twice.
   */
  template<typename Callback>
  void visit(const ScreenRect& rect, const Callback& callback) const {
    if (visited_.size() < rects_.size()) {
      visited_.resize(rects_.size(), 0U);
    }
    if (++visitCounter_ == 1U) {
      std::fill(visited_.begin(), visited_.end(), 0U);  // the counter wrapped around
    }
    for (const auto index : oversized_) {
      callback(rects_[index]);
    }
    if (!collectCells(rect)) {
      for (const auto& stored : rects_) {
        callback(stored);
      }
      return;
    }
    for (const auto& cell : scratch_) {
      const auto found = cells_.find(cell);
      if (found == cells_.end()) {
        continue;
      }
      for (const auto index : found->second) {
        if (visited_[index] == visitCounter_) {
          continue;  // the rectangle spans several cells and was already passed
        }
        visited_[index] = visitCounter_;
        callback(rects_[index]);
      }
    }
  }

private:
  using Cell = std::pair<int, int>;

  //! Rectangles spanning more cells than this are not worth indexing.
  static constexpr size_t MAX_CELLS_PER_RECT = 16;

  //! Coordinates further from the view than this are not considered part of it (they would overflow the cell index).
  static constexpr double MAX_COORDINATE = 1e7;

  //! Index of the cell the given coordinate lies in.
  int cellIndex(double coordinate) const {
    return static_cast<int>(std::floor(coordinate / cellSize_));
  }

  /**
   * \brief Collects the cells the given rectangle overlaps into scratch_.
   * \return false if the rectangle is not finite or spans too many cells, so that it cannot be located in the grid.
   */
  bool collectCells(const ScreenRect& rect) const {
    scratch_.clear();
    if (!std::isfinite(rect.x) || !std::isfinite(rect.y) || !std::isfinite(rect.w) || !std::isfinite(rect.h) ||
        std::abs(rect.x) > MAX_COORDINATE || std::abs(rect.y) > MAX_COORDINATE ||
        std::abs(rect.x + rect.w) > MAX_COORDINATE || std::abs(rect.y + rect.h) > MAX_COORDINATE)
    {
      return false;
    }
    const auto x0 = cellIndex(rect.x);
    const auto x1 = cellIndex(rect.x + rect.w);
    const auto y0 = cellIndex(rect.y);
    const auto y1 = cellIndex(rect.y + rect.h);
    if (static_cast<int64_t>(x1 - x0) * (y1 - y0) > 4 * MAX_CELLS_PER_RECT) {
      return false;
    }
    scratch_.reserve(static_cast<size_t>(x1 - x0 + 1) * static_cast<size_t>(y1 - y0 + 1));
    for (auto x = x0; x <= x1; ++x) {
      for (auto y = y0; y <= y1; ++y) {
        scratch_.push_back({x, y});
      }
    }
    return true;
  }

  double cellSize_;  //!< Size of one grid cell [px].
  std::vector<ScreenRect> rects_;  //!< All the stored rectangles.
  std::vector<uint32_t> oversized_;  //!< Indices of the rectangles which are not in the grid.
  std::map<Cell, std::vector<uint32_t>> cells_;  //!< Cell to indices of the rectangles which overlap it.
  mutable std::vector<Cell> scratch_;  //!< Reusable buffer of the cells currently examined.
  mutable std::vector<uint32_t> visited_;  //!< Visit stamp of the rectangles, to pass each of them only once.
  mutable uint32_t visitCounter_ {0};  //!< Current visit stamp.
};

//! Area of the given rectangle that is already taken by the indexed rectangles.
double overlap(const ScreenRect& rect, const RectIndex& indexed) {
  auto area = 0.;
  indexed.visit(rect, [&rect, &area](const ScreenRect& other) {
        area += intersect(rect, other).area();
  });
  return area;
}

//! Checks whether the rectangle does not overlap any of the indexed rectangles.
bool isFree(const ScreenRect& rect, const RectIndex& indexed) {
  bool free = true;
  indexed.visit(rect, [&rect, &free](const ScreenRect& other) {
        if (free && intersect(rect, other).area() > 0.) {
          free = false;
        }
  });
  return free;
}

//! Fraction of the rectangle that lies in the view.
double coverage(const ScreenRect& rect, const ScreenRect& view) {
  const auto rectArea = rect.area();
  if (rectArea <= 0.) {
    return 0.;
  }
  return intersect(rect, view).area() / rectArea;
}

}  // namespace

double estimateLabelWidth(size_t characterCount, double fontHeight) {
  // The labels are drawn with a proportional font, but half an em per character is a good enough guess for the
  // placement (and it does not need to ask the font, which would have to be initialised for that).
  return static_cast<double>(characterCount) * fontHeight * LABEL_WIDTH_FACTOR;
}

std::vector<PlacedItem> LayoutPlanner::plan(std::vector<LayoutItem> items, double viewWidth, double viewHeight) {
  return plan(std::move(items), viewWidth, viewHeight, Params());
}

std::vector<PlacedItem> LayoutPlanner::plan(std::vector<LayoutItem> items, double viewWidth, double viewHeight,
    const Params& params) {
  const ScreenRect view{0., 0., viewWidth, viewHeight};

  // The most important detections get the best place; the less important ones have to be happy with what is left.
  std::stable_sort(items.begin(), items.end(), [](const LayoutItem& a, const LayoutItem& b) {
        if (a.priority != b.priority) {
          return a.priority > b.priority;
        }
        return a.areaWeight > b.areaWeight;
  });

  // The annotations are much smaller than the view, so a grid with the cells as large as the largest annotation keeps
  // the number of rectangles examined for each candidate place small.
  auto annotationSize = params.minPreviewWidth;
  for (const auto& item : items) {
    annotationSize = std::max(annotationSize, std::max(item.maxWidth, item.labelWidth));
  }
  const auto cellSize = std::min(512., std::max(32., annotationSize));

  std::vector<PlacedItem> placed;
  placed.reserve(items.size());
  RectIndex occupied{cellSize};  //!< Annotations that were already placed (annotations must not overlap).
  RectIndex labels{cellSize};  //!< Placed labels (two labels must never overlap, that is what makes them readable).
  RectIndex markers{cellSize};  //!< Markers of the detections (annotations may cover them, but should not).
  size_t previews = 0;  //!< How many image previews are shown so far.

  for (const auto& item : items) {
    // The marker of the detection itself is always shown, so the annotations should not cover it.
    const auto marker = expanded(item.anchor, params.margin);
    const auto labelHeight = std::max(0., item.extraHeight);
    const auto labelWidth = item.labelWidth > 0. ? item.labelWidth : item.maxWidth;

    PlacedItem result;
    result.id = item.id;
    result.rect = item.anchor;

    ScreenRect bestRect;
    AnchorSide bestSide = AnchorSide::CENTER;
    bool found = false;

    // Every preview needs its own texture, so only the most important detections get one (the items are processed in
    // the order of decreasing importance).
    const bool allowPreview = item.hasPreview && (params.maxPreviews == 0 || previews < params.maxPreviews);

    // Try the full-size preview first. If it does not fit anywhere, shrink it and try again - a smaller image is
    // still much more useful than no image at all. This is also what makes previews appear when the user zooms in.
    const auto scales = std::max(1, params.previewScales);
    for (int scale = 0; allowPreview && scale < scales && !found; ++scale) {
      const double width = item.maxWidth * std::pow(0.6, scale);
      if (width < params.minPreviewWidth) {
        break;  // the preview would not be recognizable anymore
      }
      const double height = width / (item.aspectRatio > 0. ? item.aspectRatio : 1.);

      // Pick the place where the preview overlaps the least number of other detection markers. Covering a marker is
      // allowed (the detections are often so close to each other that it is not possible to avoid it), but the
      // annotations must never overlap each other.
      auto bestCost = std::numeric_limits<double>::max();
      for (const auto side : IMAGE_SIDES) {
        const auto rect = anchoredRect(item.anchor, side, width, height, params.margin);
        // The label under the preview belongs to it as well, so it needs its space too.
        const ScreenRect annotation{rect.x, rect.y, rect.w, rect.h + labelHeight};
        if (!isFree(annotation, occupied) || coverage(annotation, view) < params.minCoverage) {
          continue;
        }
        const auto cost = overlap(annotation, markers);
        if (cost >= bestCost) {
          continue;
        }
        bestRect = rect;
        bestSide = side;
        bestCost = cost;
        found = true;
        if (cost <= 0.) {
          break;  // cannot do better than this
        }
      }
    }

    if (found) {
      result.preview = true;
      result.anchor = bestSide;
      result.rect = bestRect;
      if (labelHeight > 0.) {
        result.labelRect = {bestRect.x, bestRect.y + bestRect.h, bestRect.w, labelHeight};
      }
      const ScreenRect annotation{bestRect.x, bestRect.y, bestRect.w, bestRect.h + labelHeight};
      occupied.insert(expanded(annotation, params.margin));
      if (labelHeight > 0.) {
        labels.insert(expanded(result.labelRect, params.margin));
      }
      ++previews;
    } else {
      // There is no space for an image preview, so the detection is only plotted as a plain marker. Its label is much
      // smaller than a preview, so there is usually still some free place for it - and unlike the previews, a label can
      // afford to cover a marker or an image a bit. It is never placed over another label though, because overlapping
      // texts cannot be read at all.
      if (labelHeight > 0.) {
        for (const auto side : MARKER_SIDES) {
          const auto rect = anchoredRect(item.anchor, side, labelWidth, labelHeight, params.margin);
          const auto covered = overlap(rect, occupied) + overlap(rect, markers);
          const bool fits = isFree(rect, labels) && coverage(rect, view) >= params.minCoverage &&
                            covered <= params.maxLabelOverlap * rect.area();
          if (!fits) {
            continue;
          }
          result.labelRect = rect;
          occupied.insert(expanded(rect, params.margin));
          labels.insert(expanded(rect, params.margin));
          break;
        }
      }
    }

    placed.push_back(result);
    // The annotations of the other detections should not cover this marker, but they can if there is no other place.
    markers.insert(marker);
  }

  return placed;
}

}  // namespace cras_rviz_vision_msgs
