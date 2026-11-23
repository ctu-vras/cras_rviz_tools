// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Implementation of the placement of the detection annotations in the RViz view.
 * \author Czech Technical University in Prague
 */

#include "cras_rviz_vision_msgs/layout_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <utility>

namespace cras_rviz_vision_msgs {

namespace {

//! How many times further from the detection the layout looks for a free place.
constexpr int SEARCH_RINGS = 8;

//! How many pixels are kept between a detection (or its preview) and the text label above it.
constexpr double LABEL_GAP = 2.;

}  // namespace

double estimateLabelWidth(size_t characterCount, double fontHeight) {
  // The exact width is known only to the font, but the layout only needs to know how much space to keep for it. The
  // factor was measured with the default RViz font: its lower-case letters are about a third of their height wide, and
  // the labels the display writes are short words with a few digits and spaces between them.
  return static_cast<double>(characterCount) * .35 * fontHeight;
}

std::pair<double, double> fitIntoBox(double boxWidth, double boxHeight, double aspectRatio) {
  const auto wide = std::max(0., boxWidth);
  // A box without a height of its own limits the picture the same way in both directions.
  const auto high = boxHeight > 0. ? boxHeight : wide;
  // A picture whose shape is not known is shown as a square one rather than as a line.
  const auto ratio = aspectRatio > 0. ? aspectRatio : 1.;
  if (wide <= 0. || high <= 0.) {
    return {0., 0.};
  }
  // A picture is as wide as the box it belongs to, and only becomes narrower when it would not fit into it vertically.
  // That is the case of a picture much taller than wide (a person standing), which would need a preview as tall as a
  // multiple of the view otherwise.
  const auto heightLimited = high * ratio;
  const auto widthLimited = wide / ratio;
  if (widthLimited <= high) {
    return {wide, widthLimited};
  }
  return {heightLimited, high};
}

ScreenLayout::ScreenLayout(double viewWidth, double viewHeight, const LayoutParams& params)
    : viewWidth_(viewWidth), viewHeight_(viewHeight), params_(params) {
  const auto cell = std::max(1, params_.cell);
  columns_ = std::max(1, static_cast<int>(std::ceil(std::max(1., viewWidth) / cell)));
  rows_ = std::max(1, static_cast<int>(std::ceil(std::max(1., viewHeight) / cell)));
  occupied_.assign(static_cast<size_t>(columns_) * static_cast<size_t>(rows_), -1);
  blocking_.assign(occupied_.size(), 0);
}

const std::vector<std::pair<int, int>>& ScreenLayout::searchPattern() {
  // The offsets are tried in rings around the natural place of the annotation, so that the closest free place wins.
  static const std::vector<std::pair<int, int>> pattern = []() {
                                                            std::vector<std::pair<int, int>> offsets {std::make_pair(0,
        0)};
                                                            for (int ring = 1; ring <= SEARCH_RINGS; ++ring) {
                                                              for (int direction = 0; direction < 8; ++direction) {
                                                                const auto angle = direction * M_PI / 4.;
                                                                offsets.emplace_back(std::lround(std::cos(angle) *
          ring), std::lround(std::sin(angle) * ring));
                                                              }
                                                            }
                                                            return offsets;
                                                          }();
  return pattern;
}

bool ScreenLayout::inView(const ScreenRect& rect) const {
  if (rect.w <= 0. || rect.h <= 0.) {
    return false;
  }
  // The rectangle may hang out of the view a bit, but it has to be mostly visible to be worth showing.
  const auto visible = ScreenRect {std::max(0., rect.x), std::max(0., rect.y),
    std::min(viewWidth_, rect.x + rect.w) - std::max(0., rect.x),
    std::min(viewHeight_, rect.y + rect.h) - std::max(0., rect.y)};
  return visible.area() >= params_.minCoverage * rect.area();
}

bool ScreenLayout::free(const ScreenRect& rect, int owner, const ScreenRect* ignore) const {
  if (!inView(rect)) {
    return false;
  }
  const auto cell = static_cast<double>(std::max(1, params_.cell));
  const auto firstX = std::max(0, static_cast<int>(std::floor(rect.x / cell)));
  const auto firstY = std::max(0, static_cast<int>(std::floor(rect.y / cell)));
  const auto lastX = std::min(columns_ - 1, static_cast<int>(std::floor((rect.x + rect.w) / cell)));
  const auto lastY = std::min(rows_ - 1, static_cast<int>(std::floor((rect.y + rect.h) / cell)));
  for (auto y = firstY; y <= lastY; ++y) {
    for (auto x = firstX; x <= lastX; ++x) {
      if (ignore) {
        const double cx = (x + .5) * cell;
        const double cy = (y + .5) * cell;
        if (cx >= ignore->x && cx < ignore->x + ignore->w && cy >= ignore->y && cy < ignore->y + ignore->h) {
          continue;  // the place of the annotation itself is not in the way of its own connecting line
        }
      }
      const auto index = static_cast<size_t>(y) * columns_ + x;
      if (blocking_[index] && occupied_[index] != -1 && occupied_[index] != owner) {
        return false;
      }
    }
  }
  return true;
}

ScreenRect ScreenLayout::stemRect(const ScreenRect& anchor, const ScreenRect& place, double thickness) {
  const auto cx = anchor.x + .5 * anchor.w;
  const auto cy = anchor.y + .5 * anchor.h;
  const auto px = place.x + .5 * place.w;
  const auto py = place.y + .5 * place.h;
  if (std::abs(px - cx) <= .5 * (anchor.w + place.w) && std::abs(py - cy) <= .5 * (anchor.h + place.h)) {
    return {};  // the marker is right at the preview, so no line is needed to say which detection it belongs to
  }
  // The place the line needs is the rectangle around it, widened by its thickness on each side. A steep line needs a
  // tall and narrow place, a flat one the other way round, so the two directions are kept apart.
  const auto x0 = std::min(cx, px) - thickness;
  const auto x1 = std::max(cx, px) + thickness;
  const auto y0 = std::min(cy, py) - thickness;
  const auto y1 = std::max(cy, py) + thickness;
  return {x0, y0, x1 - x0, y1 - y0};
}

ScreenRect ScreenLayout::spaced(const ScreenRect& rect) const {
  // The spacing is kept by reserving more space than the annotation itself covers, so that two neighbouring
  // annotations never touch each other.
  const auto pad = std::max(0., params_.margin);
  return {rect.x - pad, rect.y - pad, rect.w + 2. * pad, rect.h + 2. * pad};
}

void ScreenLayout::reserve(const ScreenRect& rect, int owner, bool blocks) {
  if (rect.w <= 0. || rect.h <= 0.) {
    return;
  }
  const auto cell = static_cast<double>(std::max(1, params_.cell));
  const auto firstX = std::max(0, static_cast<int>(std::floor(rect.x / cell)));
  const auto firstY = std::max(0, static_cast<int>(std::floor(rect.y / cell)));
  const auto lastX = std::min(columns_ - 1, static_cast<int>(std::floor((rect.x + rect.w) / cell)));
  const auto lastY = std::min(rows_ - 1, static_cast<int>(std::floor((rect.y + rect.h) / cell)));
  for (auto y = firstY; y <= lastY; ++y) {
    for (auto x = firstX; x <= lastX; ++x) {
      const auto index = static_cast<size_t>(y) * columns_ + x;
      occupied_[index] = owner;
      // A marker only claims its own place, it never keeps another annotation away - see free().
      if (blocks) {
        blocking_[index] = 1;
      }
    }
  }
}

bool ScreenLayout::findPlace(const ScreenRect& anchor, const ScreenRect& natural, double maxShift, double stem,
    int owner, ScreenRect& result) {
  // The search step grows with the size of the annotation, so that the place on the other side of a crowded cluster
  // is reached in a reasonable number of steps too.
  const auto step = std::max(params_.margin, .5 * std::min(natural.w, natural.h));
  for (const auto& offset : searchPattern()) {
    const auto ring = std::max(std::abs(offset.first), std::abs(offset.second));
    if (static_cast<double>(ring) * step > maxShift) {
      break;  // the following places are even further from the detection
    }
    const ScreenRect candidate {natural.x + offset.first * step, natural.y + offset.second * step, natural.w,
      natural.h};
    if (stem > 0. && !free(stemRect(anchor, candidate, stem), owner, &candidate)) {
      continue;  // the line to the marker would cover another annotation
    }
    if (free(candidate, owner)) {
      result = candidate;
      return true;
    }
  }
  return false;
}

bool ScreenLayout::findVisiblePlace(const ScreenRect& natural, double maxShift, ScreenRect& result) const {
  const auto step = std::max(params_.margin, .5 * std::min(natural.w, natural.h));
  for (const auto& offset : searchPattern()) {
    const auto ring = std::max(std::abs(offset.first), std::abs(offset.second));
    if (static_cast<double>(ring) * step > maxShift) {
      break;  // the following places are even further from the detection
    }
    const ScreenRect candidate {natural.x + offset.first * step, natural.y + offset.second * step, natural.w,
      natural.h};
    if (inView(candidate)) {
      result = candidate;
      return true;
    }
  }
  return false;
}

void ScreenLayout::place(std::vector<LayoutItem>& items) {
  std::fill(occupied_.begin(), occupied_.end(), -1);
  std::fill(blocking_.begin(), blocking_.end(), 0);
  shownPreviews_.clear();
  labelCount_ = 0;
  for (size_t item = 0; item < items.size(); ++item) {
    // The marker of a detection is drawn in any case, so it knows its own place; it does not block anything.
    reserve(items[item].anchor, static_cast<int>(item), false);
  }
  std::vector<size_t> order(items.size());
  for (size_t item = 0; item < items.size(); ++item) {
    order[item] = item;
  }
  std::stable_sort(order.begin(), order.end(), [&items](size_t a, size_t b) {
        return items[a].previewRank() > items[b].previewRank();
    });
  // The image previews go first, so that the free space is used by the most informative annotations first. If the
  // labels were placed in the same round, the detections which got their label first would block the previews of the
  // detections which come later in the order.
  for (const auto item : order) {
    auto& entry = items[item];
    if (entry.previewWidth <= 0. || entry.previewHeight <= 0. || shownPreviews_.size() >= params_.maxPreviews) {
      continue;
    }
    // The picture belongs right onto the place of its detection, and an annotation which would end up too far from it
    // is rather left out - the user could not tell which detection it belongs to anyway. The allowed distance grows
    // with the size of the annotation because a big one has to move further to get around its neighbours, and in a
    // dense cluster the previews have to spread out to avoid each other.
    const ScreenRect natural {entry.anchor.x + .5 * (entry.anchor.w - entry.previewWidth),
      entry.anchor.y + .5 * (entry.anchor.h - entry.previewHeight), entry.previewWidth, entry.previewHeight};
    const auto maxShift = 1.5 * std::max(entry.previewWidth, entry.previewHeight);
    if (findPlace(entry.anchor, natural, maxShift, entry.stem, static_cast<int>(item), entry.preview)) {
      entry.previewPlaced = true;
      entry.stemPlaced = entry.stem > 0.;
      reserve(spaced(entry.preview), static_cast<int>(item));
      if (entry.stemPlaced) {
        // The line to the marker keeps the other annotations away, so that it stays readable which preview belongs to
        // which detection even in a dense scene.
        reserve(stemRect(entry.anchor, entry.preview, std::max(entry.stem, params_.margin)),
                static_cast<int>(item));
      }
      shownPreviews_.push_back(item);
      continue;
    }
    if (entry.stem > 0. && findPlace(entry.anchor, natural, maxShift, 0., static_cast<int>(item), entry.preview)) {
      // A preview which is at least not covering another one is still worth showing; the line to its marker is then
      // drawn over whatever is in its way (it is drawn last, so it stays visible anyway).
      entry.previewPlaced = true;
      reserve(spaced(entry.preview), static_cast<int>(item));
      shownPreviews_.push_back(item);
      continue;
    }
    if (entry.selected && findVisiblePlace(natural, maxShift, entry.preview)) {
      // The detection the user asked for is shown even when the view has no free room for its picture: the display
      // draws it over the previews which are in its way. Only a picture which would be a fragment outside of the view
      // is left out - the layout cannot make something recognisable out of it.
      entry.previewPlaced = true;
      reserve(spaced(entry.preview), static_cast<int>(item));
      shownPreviews_.push_back(item);
    }
  }
  for (const auto item : order) {
    auto& entry = items[item];
    if (entry.labelWidth <= 0. || entry.labelHeight <= 0. || labelCount_ >= params_.maxLabels) {
      continue;
    }
    // A label is a caption of the picture which shows the detection, so it is written to its top; a label which floats
    // away from its detection starts being the label of whichever detection happens to be next to it. The text over its
    // own picture costs no room around it, which is what keeps the view readable when there is a crowd of detections.
    // Without a picture the label goes above the marker, and if there is no room there either, beside it.
    const auto& owned = entry.previewPlaced ? entry.preview : entry.anchor;
    const ScreenRect natural {owned.x + .5 * (owned.w - entry.labelWidth),
      entry.previewPlaced ? owned.y + LABEL_GAP : owned.y - entry.labelHeight - LABEL_GAP, entry.labelWidth,
      entry.labelHeight};
    const ScreenRect beside {owned.x + owned.w + params_.margin,
      owned.y + .5 * (owned.h - entry.labelHeight), entry.labelWidth, entry.labelHeight};
    if (findPlace(entry.anchor, natural, 3. * entry.labelHeight, 0., static_cast<int>(item), entry.label) ||
        findPlace(entry.anchor, beside, 4. * entry.labelHeight, 0., static_cast<int>(item), entry.label))
    {
      entry.labelPlaced = true;
      reserve(spaced(entry.label), static_cast<int>(item));
      ++labelCount_;
    }
  }
}

const std::vector<size_t>& ScreenLayout::shownPreviews() const {
  return shownPreviews_;
}

size_t ScreenLayout::previewCount() const {
  return shownPreviews_.size();
}

size_t ScreenLayout::labelCount() const {
  return labelCount_;
}

}  // namespace cras_rviz_vision_msgs
