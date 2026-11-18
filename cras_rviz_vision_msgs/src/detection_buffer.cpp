// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Implementation of the buffer of visualized detections.
 * \author Czech Technical University in Prague
 */

#include "cras_rviz_vision_msgs/detection_buffer.hpp"

#include <cmath>
#include <utility>

namespace cras_rviz_vision_msgs {

namespace {

//! Number of nanoseconds in a second.
constexpr double SECONDS_TO_NANOSECONDS = 1000000000.;

}  // namespace

DetectionBuffer::DetectionBuffer()
    : DetectionBuffer(DEFAULT_LIMIT) {
}

DetectionBuffer::DetectionBuffer(size_t limit)
    : limit_(limit) {
}

size_t DetectionBuffer::limit() const {
  return limit_;
}

void DetectionBuffer::setLimit(size_t limit) {
  limit_ = limit;
  while (limit_ > 0 && detections_.size() > limit_) {
    popOldest();
  }
}

std::string DetectionBuffer::key(const StoredDetection& detection) {
  if (detection.id.empty()) {
    return {};
  }
  // The same ID can appear in several frames, so the frame is part of the identity.
  return detection.frameId + '\1' + detection.id;
}

void DetectionBuffer::popOldest() {
  if (detections_.empty()) {
    return;
  }
  const auto id = key(detections_.front());
  if (!id.empty()) {
    const auto indexed = byId_.find(id);
    if (indexed != byId_.end() && indexed->second == &detections_.front()) {
      byId_.erase(indexed);
    }
  }
  detections_.pop_front();
  ++dropped_;
}

void DetectionBuffer::add(std::vector<StoredDetection> detections, bool accumulate) {
  if (!accumulate && !detections.empty()) {
    // The caller wants to show only the newest message, but an empty message must not clear the scene.
    clear();
  }
  for (auto& detection : detections) {
    const auto id = key(detection);
    const auto known = !id.empty() ? byId_.find(id) : byId_.end();
    if (known != byId_.end()) {
      // The same detection was received again, so update it in place. It keeps its serial number (and therefore also
      // its image preview) and its place in the buffer, so that a repeatedly published object does not push the other
      // detections out of the buffer.
      auto* stored = known->second;
      detection.serial = stored->serial;
      *stored = std::move(detection);
      continue;
    }
    detection.serial = ++serialCounter_;
    detections_.push_back(std::move(detection));
    if (!id.empty()) {
      byId_.insert({id, &detections_.back()});
    }
    while (limit_ > 0 && detections_.size() > limit_) {
      popOldest();
    }
  }
}

size_t DetectionBuffer::prune(int64_t nowNs, double lifetime) {
  if (!(lifetime > 0.) || !std::isfinite(lifetime)) {
    return 0;
  }
  const auto oldest = nowNs - static_cast<int64_t>(lifetime * SECONDS_TO_NANOSECONDS);
  size_t removed = 0;
  // The buffer is ordered by the time the detections were received in, so the expired ones are at its front.
  // Detections without a timestamp have no age to compare.
  while (!detections_.empty() && detections_.front().stamp.nanoseconds() != 0 &&
         detections_.front().stamp.nanoseconds() < oldest)
  {
    popOldest();
    ++removed;
  }
  return removed;
}

void DetectionBuffer::clear() {
  detections_.clear();
  byId_.clear();
  dropped_ = 0;
}

const std::deque<StoredDetection>& DetectionBuffer::detections() const {
  return detections_;
}

size_t DetectionBuffer::size() const {
  return detections_.size();
}

size_t DetectionBuffer::dropped() const {
  return dropped_;
}

}  // namespace cras_rviz_vision_msgs
