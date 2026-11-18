// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Detections which are currently visualized.
 * \author Czech Technical University in Prague
 */

#pragma once

#include <cstdint>
#include <deque>
#include <string>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/time.hpp>

#include "cras_rviz_vision_msgs/image_cache.hpp"

namespace cras_rviz_vision_msgs {

/**
 * \brief One detection to visualize, either received on the topic, or loaded from the persistent storage.
 */
struct StoredDetection {
  rclcpp::Time stamp {0, 0, RCL_ROS_TIME};  //!< Time of the detection (or of the message it arrived in).
  uint64_t serial {0};  //!< Unique identifier of this detection, assigned by whichever holds it.
  bool history {false};  //!< Whether the detection was loaded from the persistent storage.
  std::string frameId;  //!< Frame the position below is valid in (empty for the frame of the message).
  geometry_msgs::msg::Pose pose;  //!< Estimated 3D pose of the detection.
  bool hasPose {false};  //!< Whether the pose above is known.
  std::string id;  //!< ID of the detection (used to recognize repeatedly published detections).
  std::string className;  //!< The most probable class of the detection.
  double probability {1.};  //!< Probability of the most probable class.
  ImageRegion imageRegion;  //!< Which part of the source image to show as the preview of the detection.
  std::string imagePath;  //!< Image saved with the detection (stored detections only).
  bool hasZone {false};  //!< Whether the keep-out zone membership is known.
  bool inZone {false};  //!< Whether the detection is inside at least one keep-out zone.
  std::vector<int> zoneIds;  //!< Numbers of the keep-out zones the detection is in.
};

/**
 * \brief The detections received on the topic which should be visualized.
 *
 * Detections are typically published repeatedly, so neither showing only the contents of the newest message, nor
 * appending every message as they come, is useful:
 * - the detections of the older messages are kept, which allows showing more detections than a single message contains
 *   (up to the given limit) and makes an empty message harmless (the detections do not disappear just because the
 *   detector did not find anything in the last frame),
 * - a detection which has the same ID (in the same frame) as an already stored one replaces it, so that repeatedly
 *   published detections do not pile up,
 * - the oldest detections are dropped when the limit is exceeded.
 */
class DetectionBuffer {
public:
  //! Maximal number of detections kept when the limit is not set explicitly.
  static constexpr size_t DEFAULT_LIMIT = 2000;

  DetectionBuffer();
  //! \param[in] limit Maximal number of kept detections, 0 for no limit.
  explicit DetectionBuffer(size_t limit);

  DetectionBuffer(const DetectionBuffer&) = delete;
  DetectionBuffer& operator=(const DetectionBuffer&) = delete;

  size_t limit() const;
  //! Sets the maximal number of kept detections (0 for no limit) and drops the oldest ones if needed.
  void setLimit(size_t limit);

  /**
   * \brief Adds the detections of a received message.
   * \param[in] detections The detections to add; their serial numbers are assigned by the buffer.
   * \param[in] accumulate Whether to keep the detections stored so far. When false, only the given detections are kept
   *   - but the stored detections are kept even then if the given detections are empty, because an empty message
   *   should not make the visualized detections disappear.
   */
  void add(std::vector<StoredDetection> detections, bool accumulate);

  /**
   * \brief Drops the detections which are older than the given lifetime.
   *
   * Detections without a timestamp (all-zero stamp) are kept, there is nothing to compare them to.
   *
   * \param[in] nowNs Current time, in nanoseconds since the Unix epoch (in the same time base as the detections).
   * \param[in] lifetime Maximal age of the detections [s], 0 disables the pruning.
   * \return The number of dropped detections.
   */
  size_t prune(int64_t nowNs, double lifetime);

  //! Drops all detections. The number of dropped detections is reset as well.
  void clear();

  //! The kept detections, in the order they were received in (the oldest first).
  const std::deque<StoredDetection>& detections() const;
  size_t size() const;
  //! Number of detections dropped because the buffer was full or because they expired, since the last clear().
  size_t dropped() const;

private:
  //! Identity of the detection (its ID in the frame it is valid in), or an empty string if it cannot be recognized.
  static std::string key(const StoredDetection& detection);

  //! Drops the detection at the front of the buffer, together with its entry in the index.
  void popOldest();

  std::deque<StoredDetection> detections_;
  //! Index of the detections by their ID, so that a repeatedly published detection is updated instead of duplicated.
  //! The pointers stay valid because the buffer only grows and shrinks at its ends.
  std::unordered_map<std::string, StoredDetection*> byId_;
  size_t limit_;
  size_t dropped_ {0};
  uint64_t serialCounter_ {0};
};

}  // namespace cras_rviz_vision_msgs
