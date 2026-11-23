// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Implementation of the keep-out zones the detections are tested against.
 * \author Czech Technical University in Prague
 */

#include <cras_rviz_vision_msgs/keepout_zones.hpp>

#include <cmath>
#include <optional>
#include <utility>

namespace cras_rviz_vision_msgs {

namespace {

//! How far a position can be from the outline of a zone and still be considered to be on it [m].
constexpr double ON_BORDER_TOLERANCE = 1e-6;

//! How small an outline has to be to not enclose any area [m^2].
constexpr double MIN_ZONE_AREA = 1e-9;

/**
 * \brief How many edges of an outline the ray going to the right from a position crosses.
 * \param[in] zone Corners of the outline.
 * \param[in] x Horizontal position to test.
 * \param[in] y Vertical position to test.
 * \return The number of crossed edges (an odd one means the position is inside the outline), or nothing when the
 *   position lies right on the outline.
 */
std::optional<int> crossings(const KeepoutZones::Zone& zone, double x, double y) {
  int crossed = 0;
  const size_t count = zone.size();
  for (size_t i = 0, previous = count - 1; i < count; previous = i++) {
    const auto& corner = zone[i];
    const auto& neighbour = zone[previous];
    const double dx = corner.first - neighbour.first;
    const double dy = corner.second - neighbour.second;
    const double length = dx * dx + dy * dy;
    if (length <= MIN_ZONE_AREA) {
      continue;  // the outline only repeats this corner, so there is no edge to test against
    }
    // The position is on the edge when it is close to the line of the edge and not past its ends.
    const double distance = std::abs(dx * (y - neighbour.second) - dy * (x - neighbour.first));
    const double along = ((x - neighbour.first) * dx + (y - neighbour.second) * dy) / length;
    if (distance <= ON_BORDER_TOLERANCE * std::sqrt(length) && along >= 0. && along <= 1.) {
      return {};
    }
    // An edge with exactly one corner above the position lets the ray going to the right pass through it.
    if ((corner.second > y) == (neighbour.second > y)) {
      continue;
    }
    if (x < neighbour.first + (y - neighbour.second) * dx / dy) {
      ++crossed;
    }
  }
  return crossed;
}

}  // namespace

bool KeepoutZones::set(std::string frame, std::vector<Zone> zones) {
  std::vector<Zone> usable;
  usable.reserve(zones.size());
  for (auto& zone : zones) {
    // An outline has to have at least three corners and enclose an area to tell anything about a position.
    if (zone.size() < 3) {
      continue;
    }
    double twiceArea = 0.;
    for (size_t i = 0, previous = zone.size() - 1; i < zone.size(); previous = i++) {
      twiceArea += zone[previous].first * zone[i].second - zone[i].first * zone[previous].second;
    }
    if (std::abs(twiceArea) < 2. * MIN_ZONE_AREA) {
      continue;
    }
    if (zone.front() == zone.back()) {
      zone.pop_back();  // a closed outline which repeats its first corner describes the very same zone
    }
    usable.push_back(std::move(zone));
  }
  // The zones are published repeatedly, so telling whether they really changed is what decides whether the users have
  // to evaluate them for their content again.
  const bool changed = usable != zones_ || frame != frame_;
  frame_ = std::move(frame);
  zones_ = std::move(usable);
  return changed;
}

void KeepoutZones::clear() {
  frame_.clear();
  zones_.clear();
}

std::vector<int> KeepoutZones::zonesAt(double x, double y) const {
  std::vector<int> found;
  for (size_t index = 0; index < zones_.size(); ++index) {
    if (inside(zones_[index], x, y)) {
      found.push_back(static_cast<int>(index));
    }
  }
  return found;
}

bool KeepoutZones::inside(const Zone& zone, double x, double y) {
  if (zone.size() < 3) {
    return false;
  }
  const auto count = crossings(zone, x, y);
  // A position on the border of a zone belongs to it - otherwise it would flip between being in and out of it with the
  // centimetre it happens to be reported at.
  return !count || *count % 2 == 1;
}

}  // namespace cras_rviz_vision_msgs
