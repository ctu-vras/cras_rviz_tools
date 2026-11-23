// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Implementation of the buffer of visualized detections.
 * \author Czech Technical University in Prague
 */

#include "cras_rviz_vision_msgs/detection_buffer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>
#include <optional>
#include <utility>

namespace cras_rviz_vision_msgs {

namespace {

//! Number of nanoseconds in a second.
constexpr double SECONDS_TO_NANOSECONDS = 1000000000.;

/**
 * \brief The oldest confirmation a detection of a given kind may carry to stay remembered.
 * \param[in] nowNs Current time in nanoseconds since epoch.
 * \param[in] age How long the detections of the kind are kept [s]; a non-positive value keeps them forever.
 * \return The time of the oldest confirmation which still keeps such a detection, or nothing when the detections of
 *   the kind are kept whatever their age is.
 */
std::optional<int64_t> oldestConfirmation(int64_t nowNs, double age) {
  if (!(age > 0.) || !std::isfinite(age)) {
    return {};
  }
  return nowNs - static_cast<int64_t>(age * SECONDS_TO_NANOSECONDS);
}

//! Mixing of the three coordinates into the key of a cell; a collision only makes one more detection be compared.
constexpr uint64_t CELL_HASH_X = 0x9E3779B97F4A7C15ULL;
constexpr uint64_t CELL_HASH_Y = 0xC2B2AE3D27D4EB4FULL;
constexpr uint64_t CELL_HASH_Z = 0x165667B19E3779F9ULL;

//! The preview of a detection which keeps moving is taken again at most once per this period [s].
constexpr double PREVIEW_REFRESH_PERIOD = 1.;

//! How much (as a fraction of the preview size) a detection may move before its preview is taken again.
constexpr double PREVIEW_MOVE_FRACTION = .5;

//! Distance of two times in nanoseconds, without overflowing.
int64_t timeDistance(int64_t a, int64_t b) {
  return a > b ? a - b : b - a;
}

//! How far the given point is from the rectangle of a detection bounding box (0 when it is inside it).
double distanceToRegion(const ImageRegion& region, double x, double y) {
  const auto dx = std::max({region.x - x, x - (region.x + region.width), 0.});
  const auto dy = std::max({region.y - y, y - (region.y + region.height), 0.});
  return std::sqrt(dx * dx + dy * dy);
}

}  // namespace

DetectionBuffer::DetectionBuffer()
    : DetectionBuffer(DEFAULT_LIMIT) {
}

DetectionBuffer::DetectionBuffer(size_t limit)
    : limit_(limit) {
}

size_t DetectionBuffer::limit() const{
  return limit_;
}

void DetectionBuffer::setLimit(size_t limit) {
  if (limit == limit_) {
    return;
  }
  limit_ = limit;
  while (limit_ > 0 && detections_.size() > limit_) {
    erase(detections_.begin());
    ++dropped_;
    ++revision_;
  }
}

const Deduplication& DetectionBuffer::deduplication() const{
  return deduplication_;
}

void DetectionBuffer::setDeduplication(const Deduplication& deduplication) {
  if (deduplication.maxDistance3d == deduplication_.maxDistance3d &&
      deduplication.maxDistance2d == deduplication_.maxDistance2d &&
      deduplication.maxTimeDiff == deduplication_.maxTimeDiff)
  {
    return;
  }
  deduplication_ = deduplication;
  reindex();
  ++revision_;
}

std::string DetectionBuffer::key(const StoredDetection& detection) {
  if (detection.id.empty()) {
    return {};
  }
  // The same ID can appear in several frames, and the cameras can share their IDs, so both are the identity. The frame
  // the detection arrived in is used for it, not the frame of its pose, because that one changes when the pose is moved
  // to the global frame while the detector keeps reporting the detection in the frame it came in.
  const auto& frame = detection.sourceFrameId.empty() ? detection.frameId : detection.sourceFrameId;
  return frame + '\1' + detection.cameraFrame + '\1' + detection.id;
}

uint64_t DetectionBuffer::cellKey(double x, double y, double z) const{
  const auto ix = static_cast<int64_t>(std::floor(x / cellSize()));
  const auto iy = static_cast<int64_t>(std::floor(y / cellSize()));
  const auto iz = static_cast<int64_t>(std::floor(z / cellSize()));
  return (static_cast<uint64_t>(ix) * CELL_HASH_X) ^ (static_cast<uint64_t>(iy) * CELL_HASH_Y) ^
  (static_cast<uint64_t>(iz) * CELL_HASH_Z);
}

bool DetectionBuffer::isSimilar(
    const StoredDetection& stored, const StoredDetection& fresh,
    const Deduplication& rules) {
  if (!stored.hasPose || !fresh.hasPose || stored.frameId != fresh.frameId) {
    return false;  // positions in two different frames say nothing about each other
  }
  if (!stored.cameraFrame.empty() && !fresh.cameraFrame.empty() &&
      stored.cameraFrame != fresh.cameraFrame)
  {
    return false;  // the same place seen by two cameras are two detections, each with its own image
  }
  const auto dx = stored.pose.position.x - fresh.pose.position.x;
  const auto dy = stored.pose.position.y - fresh.pose.position.y;
  const auto dz = stored.pose.position.z - fresh.pose.position.z;
  if (dx * dx + dy * dy + dz * dz > rules.maxDistance3d * rules.maxDistance3d) {
    return false;
  }
  // Two different objects may be at the same place, but not at the same place in the image.
  if (rules.maxDistance2d > 0. && stored.imageRegion.width > 0. && fresh.imageRegion.width > 0.) {
    const auto cx = stored.imageRegion.x + .5 * stored.imageRegion.width -
                    (fresh.imageRegion.x + .5 * fresh.imageRegion.width);
    const auto cy = stored.imageRegion.y + .5 * stored.imageRegion.height -
                    (fresh.imageRegion.y + .5 * fresh.imageRegion.height);
    if (cx * cx + cy * cy > rules.maxDistance2d * rules.maxDistance2d) {
      return false;
    }
  }
  // Detections far apart in time describe different moments of time, and the user wants to see all of them.
  if (rules.maxTimeDiff > 0. &&
      std::abs(stored.stamp.nanoseconds() - fresh.stamp.nanoseconds()) >
      rules.maxTimeDiff * SECONDS_TO_NANOSECONDS)
  {
    return false;
  }
  return true;
}

DetectionBuffer::Iterator DetectionBuffer::findSimilar(const StoredDetection& fresh) {
  if (!deduplication_.enabled() || !fresh.hasPose) {
    return detections_.end();
  }
  // The detections the new one may be close to are in the cell it falls into and in its 27 neighbours.
  const auto ix = static_cast<int64_t>(std::floor(fresh.pose.position.x / cellSize()));
  const auto iy = static_cast<int64_t>(std::floor(fresh.pose.position.y / cellSize()));
  const auto iz = static_cast<int64_t>(std::floor(fresh.pose.position.z / cellSize()));
  Iterator best = detections_.end();
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dz = -1; dz <= 1; ++dz) {
        const auto key = (static_cast<uint64_t>(ix + dx) * CELL_HASH_X) ^
                         (static_cast<uint64_t>(iy + dy) * CELL_HASH_Y) ^
                         (static_cast<uint64_t>(iz + dz) * CELL_HASH_Z);
        const auto& target = grid(fresh.history);
        const auto bucket = target.find(key);
        if (bucket == target.end()) {
          continue;
        }
        for (const auto& candidate : bucket->second) {
          // The newest of the matching detections is the one the new detection describes best.
          if (isSimilar(*candidate, fresh, deduplication_) &&
              (best == detections_.end() || candidate->serial > best->serial))
          {
            best = candidate;
          }
        }
      }
    }
  }
  return best;
}

void DetectionBuffer::index(Iterator detection) {
  if (!deduplication_.enabled() || !detection->hasPose) {
    return;
  }
  grid(detection->history)[cellKey(detection->pose.position.x, detection->pose.position.y,
    detection->pose.position.z)].push_back(detection);
}

void DetectionBuffer::unindex(Iterator detection) {
  if (deduplication_.enabled() && detection->hasPose) {
    auto& target = grid(detection->history);
    const auto bucket = target.find(
      cellKey(detection->pose.position.x, detection->pose.position.y, detection->pose.position.z));
    if (bucket != target.end()) {
      bucket->second.erase(
        std::remove(bucket->second.begin(), bucket->second.end(), detection), bucket->second.end());
      if (bucket->second.empty()) {
        target.erase(bucket);
      }
    }
  }
}

void DetectionBuffer::erase(Iterator detection) {
  const auto id = key(*detection);
  if (!id.empty()) {
    const auto indexed = byId_.find(id);
    if (indexed != byId_.end() && indexed->second == detection) {
      byId_.erase(indexed);
    }
  }
  unindex(detection);
  detections_.erase(detection);
}

void DetectionBuffer::reindex() {
  for (auto& grids : grids_) {
    grids.clear();
  }
  byId_.clear();
  for (auto detection = detections_.begin(); detection != detections_.end(); ++detection) {
    const auto id = key(*detection);
    if (!id.empty()) {
      byId_[id] = detection;
    }
    index(detection);
  }
}

void DetectionBuffer::adoptPreview(StoredDetection& fresh, const StoredDetection& previous) {
  // The preview belongs to the detection as such, not to the message which reported it: a detection which keeps being
  // reported (or which is replaced by an almost identical one) keeps showing the image it was first seen in.
  fresh.previewStamp = previous.previewStamp;
  fresh.previewRegion = previous.previewRegion;
  fresh.previewNs = previous.previewNs;
  fresh.previewTaken = previous.previewTaken;
  choosePreview(fresh);
}

void DetectionBuffer::choosePreview(StoredDetection& detection) {
  // The preview of a detection stays the image it was first seen in (or the image of the detection it replaced).
  // Taking it again on every message would create a new texture for every detection many times per second, which is
  // the most expensive thing the display can do. A detection which moved away from the image it shows takes a new one,
  // but not more often than once per PREVIEW_REFRESH_PERIOD.
  const bool hasRegion = detection.previewRegion.width > 0. && detection.previewRegion.height > 0.;
  if (detection.previewTaken && hasRegion && detection.imageRegion.width > 0.) {
    const auto centreX = detection.imageRegion.x + .5 * detection.imageRegion.width;
    const auto centreY = detection.imageRegion.y + .5 * detection.imageRegion.height;
    const auto movedX = centreX - (detection.previewRegion.x + .5 * detection.previewRegion.width);
    const auto movedY = centreY - (detection.previewRegion.y + .5 * detection.previewRegion.height);
    const auto size = PREVIEW_MOVE_FRACTION * std::max(detection.previewRegion.width,
        detection.previewRegion.height);
    const auto stale = static_cast<double>(detection.receivedNs - detection.previewNs) >
                       PREVIEW_REFRESH_PERIOD * SECONDS_TO_NANOSECONDS;
    if (movedX * movedX + movedY * movedY < size * size || !stale) {
      return;  // keep the preview which was taken before
    }
  }
  detection.previewStamp = detection.stamp;
  detection.previewRegion = detection.imageRegion;
  detection.previewNs = detection.receivedNs;
  detection.previewTaken = true;
}

void DetectionBuffer::add(std::vector<StoredDetection> detections, bool accumulate) {
  if (detections.empty()) {
    return;  // an empty message must not hide the detections which were received before
  }
  if (!accumulate) {
    clearLive();
  }
  for (auto& fresh : detections) {
    if (fresh.receivedNs == 0) {
      fresh.receivedNs = fresh.stamp.nanoseconds();
    }
    const auto id = key(fresh);
    const auto known = !id.empty() ? byId_.find(id) : byId_.end();
    if (known != byId_.end()) {
      // The same detection was received again, so update it in place. It keeps its serial number (and therefore also
      // its rendering resources) and its place in the buffer, so that a repeatedly published object does not push the
      // other detections out of the buffer.
      fresh.serial = known->second->serial;
      adoptPreview(fresh, *known->second);
      unindex(known->second);  // the updated detection can be in another cell of the position index
      *known->second = std::move(fresh);
      index(known->second);
      continue;
    }
    // A detection which is (almost) at the same place, in the same part of the image and at the same time as an
    // existing one reports the same object again, so it replaces the older report. It takes over the serial number and
    // the preview of the replaced detection, so that the visualization stays the same, and it is put in its place, so
    // that the order in which the detections are remembered does not change.
    const auto replaced = findSimilar(fresh);
    Iterator inserted;
    if (replaced != detections_.end()) {
      fresh.serial = replaced->serial;
      adoptPreview(fresh, *replaced);
      inserted = detections_.insert(replaced, std::move(fresh));
    } else {
      choosePreview(fresh);
      fresh.serial = ++serialCounter_;
      detections_.push_back(std::move(fresh));
      inserted = std::prev(detections_.end());
    }
    if (!id.empty()) {
      byId_.insert({id, inserted});
    }
    index(inserted);
    if (replaced != detections_.end()) {
      erase(replaced);
      ++replaced_;
    }
    ++revision_;
  }
  while (limit_ > 0 && detections_.size() > limit_) {
    erase(detections_.begin());
    ++dropped_;
    ++revision_;
  }
}

size_t DetectionBuffer::prune(int64_t nowNs, double lifetime, double inZoneLifetime) {
  const auto anywhere = oldestConfirmation(nowNs, lifetime);
  const auto inZone = oldestConfirmation(nowNs, inZoneLifetime);
  if (!anywhere && !inZone) {
    return 0;
  }
  size_t removed = 0;
  for (auto detection = detections_.begin(); detection != detections_.end();) {
    // A detection which keeps being reported has its confirmation time updated, so it never runs out of its lifetime.
    // The detections from the storage are old on purpose, but they age the same way as the ones received on the topic -
    // the user wants to see the recent past, not the whole history. Detections without any timestamp have no age to
    // compare, so they stay.
    const auto oldest = insideZone(*detection) ? inZone : anywhere;
    if (detection->receivedNs != 0 && oldest && detection->receivedNs < *oldest) {
      erase(detection++);
      ++removed;
    } else {
      ++detection;
    }
  }
  if (removed > 0) {
    expired_ += removed;
    ++revision_;
  }
  return removed;
}

void DetectionBuffer::clear() {
  detections_.clear();
  byId_.clear();
  for (auto& grids : grids_) {
    grids.clear();
  }
  dropped_ = 0;
  replaced_ = 0;
  expired_ = 0;
  ++revision_;
}

size_t DetectionBuffer::eraseGroup(bool history) {
  const auto before = detections_.size();
  for (auto detection = detections_.begin(); detection != detections_.end();) {
    if (detection->history == history) {
      erase(detection++);
    } else {
      ++detection;
    }
  }
  const auto removed = before - detections_.size();
  if (removed > 0) {
    ++revision_;
  }
  return removed;
}

size_t DetectionBuffer::clearLive() {
  return eraseGroup(false);
}

size_t DetectionBuffer::clearHistory() {
  return eraseGroup(true);
}

std::list<StoredDetection>& DetectionBuffer::detections() {
  return detections_;
}

const std::list<StoredDetection>& DetectionBuffer::detections() const{
  return detections_;
}

size_t DetectionBuffer::size() const{
  return detections_.size();
}

size_t DetectionBuffer::dropped() const{
  return dropped_;
}

size_t DetectionBuffer::replaced() const{
  return replaced_;
}

size_t DetectionBuffer::expired() const{
  return expired_;
}

int64_t DetectionBuffer::newestStamp() const{
  auto newest = int64_t{0};
  for (const auto& detection : detections_) {
    newest = std::max(newest, std::max(detection.stamp.nanoseconds(), detection.receivedNs));
  }
  return newest;
}

uint64_t DetectionBuffer::revision() const{
  return revision_;
}

std::vector<uint64_t> DetectionBuffer::select(const Selection& selection) const{
  const auto found = select(selection, selection.hasPoint, false);
  if (!found.empty() || !selection.hasPoint) {
    return found;
  }
  // The detections do not have to know their place in the image (a detector often reports them in a frame of the
  // robot), and a click in a camera image should not be wasted then - the closest detection of that camera and time is
  // the one the user meant, and it is better to highlight it than to show nothing at all.
  return select(selection, false, true);
}

std::vector<uint64_t> DetectionBuffer::select(
    const Selection& selection, bool usePlace,
    bool requireCamera) const{
  const auto tolerance = std::max(0., selection.tolerance) * SECONDS_TO_NANOSECONDS;
  const auto maxDistance = std::max(0., selection.maxDistance);
  // An image without a stamp says nothing about the time, so the newest detection which fits is the one to show.
  const auto anyTime = selection.stamp.nanoseconds() == 0;
  // How likely each candidate is: the camera it was reported by, the distance of the click from its bounding box, the
  // size of that bounding box (the user clicked on something they could see) and the distance of their stamps.
  std::vector<std::array<double, 4>> rating;
  std::vector<uint64_t> serials;
  for (const auto& detection : detections_) {
    auto distance = 0.;
    if (usePlace) {
      if (detection.imageRegion.width <= 1. || detection.imageRegion.height <= 1.) {
        continue;  // a detection without a place in the image cannot be the one the user pointed at
      }
      // The clicked place is known for the image the clicked picture comes from, and that one does not have to be the
      // image the detection arrived in: the visualization strip the user clicks in shows a picture which can be older
      // than the newest message of the detection. The picture of the preview is a place in the image the preview was
      // taken from, so the nearer of the two places is the one the click is about.
      distance = distanceToRegion(detection.imageRegion, selection.x, selection.y);
      if (detection.previewTaken && detection.previewRegion.width > 1. &&
          detection.previewRegion.height > 1.)
      {
        distance = std::min(distance,
            distanceToRegion(detection.previewRegion, selection.x, selection.y));
      }
      if (distance > maxDistance) {
        continue;
      }
    }
    auto gap = timeDistance(detection.stamp.nanoseconds(), selection.stamp.nanoseconds());
    if (detection.previewTaken && detection.previewStamp.nanoseconds() != 0) {
      // The image the user clicked can be the one the preview of the detection was taken from, which is not necessarily
      // the image the detection arrived in (a merged detection shows several of them).
      gap = std::min(gap,
          timeDistance(detection.previewStamp.nanoseconds(), selection.stamp.nanoseconds()));
    }
    if (!anyTime && gap > tolerance) {
      continue;
    }
    // Two cameras looking at the same place see the same object at the same time, so the camera decides when they both
    // fit, but it is only a preference - the detections do not have to be reported in the frames of their cameras.
    const auto otherCamera = !selection.cameraFrame.empty() && !detection.cameraFrame.empty() &&
                             selection.cameraFrame != detection.cameraFrame;
    if (requireCamera && otherCamera) {
      continue;  // the user cannot have clicked on an object of another camera than the image they clicked in
    }
    const auto area = detection.imageRegion.width * detection.imageRegion.height;
    rating.push_back({otherCamera ? 1. : 0., usePlace ? distance : 0., usePlace ? area : 0.,
          anyTime ? -static_cast<double>(detection.stamp.nanoseconds()) : static_cast<double>(gap)});
    serials.push_back(detection.serial);
  }
  std::vector<size_t> order(rating.size());
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(), [&rating](size_t a, size_t b) {
        return rating[a] < rating[b];
                                                                                                    });
  std::vector<uint64_t> selected;
  selected.reserve(order.size());
  for (const auto index : order) {
    selected.push_back(serials[index]);
  }
  return selected;
}

bool DetectionBuffer::retakePreview(uint64_t serial, const Selection& selection) {
  if (selection.stamp.nanoseconds() == 0) {
    return false;  // an image without a stamp cannot be shown
  }
  for (auto& detection : detections_) {
    if (detection.serial != serial) {
      continue;
    }
    detection.previewStamp = selection.stamp;
    // The preview shows the bounding box of the detection and the same surroundings around it as the picture of the
    // object in the visualization strip, so that the operator sees the cutout they clicked on. The crop therefore stays
    // with the bounding box when the click is on the object. It only moves to the clicked place when the user pointed
    // beside the box, i.e. at another object of the picture - and the place of the click is then the only place known
    // to be in the clicked image.
    auto region = detection.imageRegion;
    const auto width = region.width > 1. ? region.width : detection.previewRegion.width;
    const auto height = region.height > 1. ? region.height : detection.previewRegion.height;
    if (selection.hasPoint && width > 1. && height > 1.) {
      const auto onObject = selection.x >= region.x && selection.x <= region.x + width &&
                            selection.y >= region.y && selection.y <= region.y + height;
      const auto centreX = onObject ? region.x + .5 * width : selection.x;
      const auto centreY = onObject ? region.y + .5 * height : selection.y;
      region = {centreX - .5 * width, centreY - .5 * height, width, height};
    }
    detection.previewRegion = region;
    detection.previewNs = detection.receivedNs;
    detection.previewTaken = true;
    return true;
  }
  return false;
}

}  // namespace cras_rviz_vision_msgs
