// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Tests of the buffer of the visualized detections.
 * \author Czech Technical University in Prague
 */

#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

#include "cras_rviz_vision_msgs/detection_buffer.hpp"

namespace {

using cras_rviz_vision_msgs::DetectionBuffer;
using cras_rviz_vision_msgs::StoredDetection;

const int64_t SECOND = 1000000000LL;

/**
 * \brief Message with the detections of the given IDs, all at the same time.
 * \param[in] nanoseconds Stamp of the message.
 * \param[in] ids IDs of the detections; an empty ID makes an unrecognized detection.
 */
std::vector<StoredDetection> makeMessage(int64_t nanoseconds, const std::vector<std::string>& ids) {
  std::vector<StoredDetection> detections;
  for (const auto& id : ids) {
    StoredDetection detection;
    detection.stamp = rclcpp::Time(nanoseconds, RCL_ROS_TIME);
    detection.frameId = "map";
    detection.id = id;
    detection.className = "person";
    detection.probability = .5;
    detection.pose.position.x = static_cast<double>(detections.size());
    detection.hasPose = true;
    detections.push_back(std::move(detection));
  }
  return detections;
}

}  // namespace

TEST(DetectionBuffer, KeepsDetectionsOfOlderMessages)
{
  DetectionBuffer buffer;

  auto first = makeMessage(1 * SECOND, {"a", "b"});
  buffer.add(first, true);
  ASSERT_EQ(buffer.size(), 2U);

  // An empty message must not hide the detections, a detector publishing nothing is a normal thing.
  std::vector<StoredDetection> empty;
  buffer.add(empty, true);
  EXPECT_EQ(buffer.size(), 2U);

  buffer.add(makeMessage(2 * SECOND, {"c"}), true);
  ASSERT_EQ(buffer.size(), 3U);
  EXPECT_EQ(buffer.detections()[0].id, "a");
  EXPECT_EQ(buffer.detections()[1].id, "b");
  EXPECT_EQ(buffer.detections()[2].id, "c");
  EXPECT_EQ(buffer.detections()[2].stamp.nanoseconds(), 2 * SECOND);
}

TEST(DetectionBuffer, UpdatesRepeatedDetectionsInPlace)
{
  DetectionBuffer buffer;

  buffer.add(makeMessage(1 * SECOND, {"a", "b"}), true);
  auto repeated = makeMessage(2 * SECOND, {"b", "c"});
  repeated[0].pose.position.x = 21.;  // the same detection, but it moved
  repeated[0].probability = .9;
  buffer.add(repeated, true);

  // The detection which was published again keeps its place (so that a tracked object does not push the other ones
  // out of the buffer) but carries the new data.
  ASSERT_EQ(buffer.size(), 3U);
  EXPECT_EQ(buffer.detections()[0].id, "a");
  EXPECT_EQ(buffer.detections()[1].id, "b");
  EXPECT_EQ(buffer.detections()[1].stamp.nanoseconds(), 2 * SECOND);
  EXPECT_DOUBLE_EQ(buffer.detections()[1].pose.position.x, 21.);
  EXPECT_DOUBLE_EQ(buffer.detections()[1].probability, .9);
  EXPECT_EQ(buffer.detections()[2].id, "c");
}

TEST(DetectionBuffer, DoesNotRecognizeDetectionsWithoutId)
{
  DetectionBuffer buffer;

  buffer.add(makeMessage(1 * SECOND, {"", ""}), true);
  buffer.add(makeMessage(2 * SECOND, {"", ""}), true);

  EXPECT_EQ(buffer.size(), 4U);  // nothing to tell them apart, so all of them are shown
}

TEST(DetectionBuffer, RecognizesIdsOnlyWithinTheFrame)
{
  DetectionBuffer buffer;

  buffer.add(makeMessage(1 * SECOND, {"a"}), true);
  auto otherFrame = makeMessage(2 * SECOND, {"a"});
  otherFrame[0].frameId = "camera_link";
  buffer.add(otherFrame, true);

  // The same ID in another frame is another object (and cannot be placed at the same position).
  EXPECT_EQ(buffer.size(), 2U);
}

TEST(DetectionBuffer, ReplacesEverythingWhenNotAccumulating)
{
  DetectionBuffer buffer;

  buffer.add(makeMessage(1 * SECOND, {"a", "b"}), true);
  buffer.add(makeMessage(2 * SECOND, {"c"}), false);
  EXPECT_EQ(buffer.size(), 1U);
  EXPECT_EQ(buffer.detections()[0].id, "c");

  // ... but still not when the message carries nothing.
  std::vector<StoredDetection> empty;
  buffer.add(empty, false);
  EXPECT_EQ(buffer.size(), 1U);
  EXPECT_EQ(buffer.detections()[0].id, "c");
}

TEST(DetectionBuffer, DropsOldestDetectionsOverTheLimit)
{
  DetectionBuffer buffer(3);

  buffer.add(makeMessage(1 * SECOND, {"a", "b", "c"}), true);
  ASSERT_EQ(buffer.size(), 3U);
  EXPECT_EQ(buffer.dropped(), 0U);

  buffer.add(makeMessage(2 * SECOND, {"d"}), true);
  ASSERT_EQ(buffer.size(), 3U);
  EXPECT_EQ(buffer.dropped(), 1U);
  EXPECT_EQ(buffer.detections()[0].id, "b");
  EXPECT_EQ(buffer.detections()[2].id, "d");

  // A lot of detections at once, as it happens when the detector sends its whole scene periodically.
  buffer.add(makeMessage(3 * SECOND, {"e", "f", "g", "h"}), true);
  ASSERT_EQ(buffer.size(), 3U);
  EXPECT_EQ(buffer.detections()[0].id, "f");
  EXPECT_EQ(buffer.detections()[2].id, "h");
  EXPECT_EQ(buffer.dropped(), 5U);

  buffer.setLimit(10);
  EXPECT_EQ(buffer.limit(), 10U);
  buffer.setLimit(1);
  EXPECT_EQ(buffer.size(), 1U);
  EXPECT_EQ(buffer.detections()[0].id, "h");

  // A dropped detection can be received again, it must not stay in the index of the known IDs.
  buffer.setLimit(0);
  buffer.add(makeMessage(4 * SECOND, {"a"}), true);
  EXPECT_EQ(buffer.size(), 2U);
  EXPECT_EQ(buffer.detections()[1].id, "a");
}

TEST(DetectionBuffer, DropsExpiredDetections)
{
  DetectionBuffer buffer;

  buffer.add(makeMessage(10 * SECOND, {"a", "b"}), true);
  buffer.add(makeMessage(11 * SECOND, {"c"}), true);

  EXPECT_EQ(buffer.prune(11 * SECOND + SECOND / 2, 0.), 0U);  // the lifetime is disabled
  EXPECT_EQ(buffer.size(), 3U);

  // Only the detections older than the lifetime are dropped; they are always the oldest ones in the buffer.
  EXPECT_EQ(buffer.prune(11 * SECOND + SECOND / 2, 2.), 0U);
  EXPECT_EQ(buffer.size(), 3U);
  EXPECT_EQ(buffer.prune(11 * SECOND + SECOND / 2, 1.), 2U);
  EXPECT_EQ(buffer.size(), 1U);
  EXPECT_EQ(buffer.detections()[0].id, "c");
  EXPECT_EQ(buffer.dropped(), 2U);

  // A detection without a timestamp cannot be old, there is nothing to compare it to.
  auto timeless = makeMessage(0, {"d"});
  buffer.add(timeless, true);
  EXPECT_EQ(buffer.prune(12 * SECOND, 1.), 0U);
  EXPECT_EQ(buffer.size(), 2U);

  buffer.clear();
  EXPECT_EQ(buffer.size(), 0U);
  EXPECT_EQ(buffer.dropped(), 0U);
}

TEST(DetectionBuffer, AssignsUniqueSerialNumbers)
{
  DetectionBuffer buffer;

  buffer.add(makeMessage(1 * SECOND, {"a", "b"}), true);
  const auto first = buffer.detections()[0].serial;
  ASSERT_NE(first, 0U);  // zero is reserved for detections which were not stored yet
  EXPECT_NE(buffer.detections()[1].serial, first);

  // The serial number of a repeatedly published detection does not change, so that its image preview can be reused.
  buffer.add(makeMessage(2 * SECOND, {"a", "c"}), true);
  ASSERT_EQ(buffer.size(), 3U);
  EXPECT_EQ(buffer.detections()[0].serial, first);
  EXPECT_NE(buffer.detections()[2].serial, first);
}
