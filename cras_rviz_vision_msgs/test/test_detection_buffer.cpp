// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Tests of the buffer of the visualized detections.
 * \author Czech Technical University in Prague
 */

#include <gtest/gtest.h>

#include <cstdint>
#include <iterator>
#include <string>
#include <vector>

#include "cras_rviz_vision_msgs/detection_buffer.hpp"

namespace {

using cras_rviz_vision_msgs::DetectionBuffer;
using cras_rviz_vision_msgs::Deduplication;
using cras_rviz_vision_msgs::Selection;
using cras_rviz_vision_msgs::StoredDetection;

const int64_t SECOND = 1000000000LL;

/**
 * \brief Message with the detections of the given IDs, all at the same time.
 * \param[in] nanoseconds Stamp of the message.
 * \param[in] ids IDs of the detections; an empty ID makes an unrecognized detection.
 * \param[in] spacing Distance between the detections, so that they are not the same object.
 */
std::vector<StoredDetection> makeMessage(
    int64_t nanoseconds, const std::vector<std::string>& ids,
    double spacing = 10.) {
  std::vector<StoredDetection> detections;
  for (const auto& id : ids) {
    StoredDetection detection;
    detection.stamp = rclcpp::Time(nanoseconds, RCL_ROS_TIME);
    detection.frameId = "map";
    detection.id = id;
    detection.className = "person";
    detection.probability = .5;
    detection.pose.position.x = spacing * static_cast<double>(detections.size());
    detection.hasPose = true;
    // Position of the detection in the source image, which keeps the detections apart as much as their position.
    detection.imageRegion = {100. * static_cast<double>(detections.size()), 100., 40., 80.};
    detections.push_back(std::move(detection));
  }
  return detections;
}

//! Detection at the given place of the world and of the image, reported at the given time.
StoredDetection makeDetection(
    int64_t nanoseconds, double x, double imageX,
    const std::string& id = "") {
  auto detection = makeMessage(nanoseconds, {id})[0];
  detection.pose.position.x = x;
  detection.imageRegion.x = imageX;
  return detection;
}

//! The i-th detection in the order in which the buffer remembers them.
const StoredDetection& at(const DetectionBuffer& buffer, size_t index) {
  return *std::next(buffer.detections().begin(), index);
}

//! Makes the buffer remember every detection separately (the filter which merges them is the thing under test).
void withoutDeduplication(DetectionBuffer& buffer) {
  buffer.setDeduplication({0., 0., 0.});
}

//! The camera image the user clicked in, without a place (what the Header message says).
Selection clickedImage(const std::string& cameraFrame, int64_t nanoseconds) {
  Selection selection;
  selection.cameraFrame = cameraFrame;
  selection.stamp = rclcpp::Time(nanoseconds, RCL_ROS_TIME);
  return selection;
}

//! The place of a camera image the user clicked at, in the pixels of that image.
Selection clickedPoint(const std::string& cameraFrame, int64_t nanoseconds, double x, double y) {
  auto selection = clickedImage(cameraFrame, nanoseconds);
  selection.hasPoint = true;
  selection.x = x;
  selection.y = y;
  return selection;
}

}  // namespace

TEST(DetectionBuffer, KeepsDetectionsOfOlderMessages)
{
  DetectionBuffer buffer;
  withoutDeduplication(buffer);

  buffer.add(makeMessage(1 * SECOND, {"a", "b"}), true);
  ASSERT_EQ(buffer.size(), 2U);

  // An empty message must not hide the detections, a detector publishing nothing is a normal thing.
  std::vector<StoredDetection> empty;
  buffer.add(empty, true);
  EXPECT_EQ(buffer.size(), 2U);

  buffer.add(makeMessage(2 * SECOND, {"c"}), true);
  ASSERT_EQ(buffer.size(), 3U);
  EXPECT_EQ(at(buffer, 0).id, "a");
  EXPECT_EQ(at(buffer, 1).id, "b");
  EXPECT_EQ(at(buffer, 2).id, "c");
  EXPECT_EQ(at(buffer, 2).stamp.nanoseconds(), 2 * SECOND);
}

TEST(DetectionBuffer, DropsTheOldestDetectionsAboveTheLimit)
{
  DetectionBuffer buffer {2};
  withoutDeduplication(buffer);

  buffer.add(makeMessage(1 * SECOND, {"a", "b"}), true);
  buffer.add(makeMessage(2 * SECOND, {"c"}), true);

  ASSERT_EQ(buffer.size(), 2U);
  EXPECT_EQ(at(buffer, 0).id, "b");
  EXPECT_EQ(at(buffer, 1).id, "c");
  EXPECT_EQ(buffer.dropped(), 1U);
}

TEST(DetectionBuffer, KeepsOnlyTheNewestMessageWhenNotAccumulating)
{
  DetectionBuffer buffer;

  buffer.add(makeMessage(1 * SECOND, {"a", "b"}), true);
  buffer.add(makeMessage(2 * SECOND, {"c"}), false);

  ASSERT_EQ(buffer.size(), 1U);
  EXPECT_EQ(at(buffer, 0).id, "c");
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
  EXPECT_EQ(at(buffer, 0).id, "a");
  EXPECT_EQ(at(buffer, 1).id, "b");
  EXPECT_EQ(at(buffer, 1).stamp.nanoseconds(), 2 * SECOND);
  EXPECT_DOUBLE_EQ(at(buffer, 1).pose.position.x, 21.);
  EXPECT_DOUBLE_EQ(at(buffer, 1).probability, .9);
  EXPECT_EQ(at(buffer, 2).id, "c");
}

TEST(DetectionBuffer, KeepsSerialOfRepeatedDetection)
{
  DetectionBuffer buffer;

  buffer.add(makeMessage(1 * SECOND, {"a"}), true);
  const auto serial = at(buffer, 0).serial;
  buffer.add(makeMessage(2 * SECOND, {"a"}), true);

  // The serial number identifies the detection to the rendering resources, so it must not change when the detection is
  // only reported again.
  ASSERT_EQ(buffer.size(), 1U);
  EXPECT_EQ(at(buffer, 0).serial, serial);
}

TEST(DetectionBuffer, RecognizesIdsOnlyWithinTheFrame)
{
  DetectionBuffer buffer;
  withoutDeduplication(buffer);

  buffer.add(makeMessage(1 * SECOND, {"a"}), true);
  auto otherFrame = makeMessage(2 * SECOND, {"a"});
  otherFrame[0].frameId = "camera_link";
  buffer.add(otherFrame, true);

  // The same ID in another frame is another object (and cannot be placed at the same position).
  EXPECT_EQ(buffer.size(), 2U);
}

TEST(DetectionBuffer, MergesDetectionsWhichAreCloseInEveryWay)
{
  DetectionBuffer buffer;

  buffer.add({makeDetection(1 * SECOND, 10., 100., "a")}, true);
  const auto serial = at(buffer, 0).serial;
  // A detection 30 cm away, 20 px away in the image and 1 s away in time is the same object seen again.
  auto again = makeDetection(2 * SECOND, 10.3, 120., "b");
  again.probability = .8;
  buffer.add({again}, true);

  ASSERT_EQ(buffer.size(), 1U);
  EXPECT_EQ(at(buffer, 0).probability, .8);  // the newer report won
  EXPECT_EQ(at(buffer, 0).serial, serial);  // and it took the place and identity of the older one
  EXPECT_EQ(buffer.replaced(), 1U);
}

TEST(DetectionBuffer, KeepsDetectionsWhichAreFarApart)
{
  DetectionBuffer buffer;

  // The default threshold is 1 m, 50 px and 5 s.
  buffer.add({makeDetection(1 * SECOND, 10., 100.)}, true);
  buffer.add({makeDetection(1 * SECOND, 12., 100.)}, true);  // 2 m away
  buffer.add({makeDetection(1 * SECOND, 10., 300.)}, true);  // 200 px away in the image
  buffer.add({makeDetection(11 * SECOND, 10., 100.)}, true);  // 10 s away in time

  EXPECT_EQ(buffer.size(), 4U);
  EXPECT_EQ(buffer.replaced(), 0U);
}

TEST(DetectionBuffer, IgnoresTheImageDistanceOfDetectionWithoutBoundingBox)
{
  DetectionBuffer buffer;

  auto first = makeDetection(1 * SECOND, 10., 100.);
  first.imageRegion = {0., 0., 0., 0.};  // the detector does not tell where the detection is in the image
  buffer.add({first}, true);
  auto again = makeDetection(2 * SECOND, 10.1, 500.);
  again.imageRegion = {0., 0., 0., 0.};
  buffer.add({again}, true);

  // Nothing is known about the image position, so only the position and the time can tell the detections apart.
  EXPECT_EQ(buffer.size(), 1U);
}

TEST(DetectionBuffer, DoesNotMergeWhenTheThresholdsAreNotSatisfied)
{
  DetectionBuffer buffer;
  withoutDeduplication(buffer);

  buffer.add({makeDetection(1 * SECOND, 10., 100.)}, true);
  buffer.add({makeDetection(1 * SECOND, 10., 100.)}, true);  // the very same detection

  // The filter is off, so everything the detector reports is remembered (up to the limit).
  EXPECT_EQ(buffer.size(), 2U);

  buffer.setDeduplication({1., 50., 5.});
  buffer.add({makeDetection(1 * SECOND, 10., 100.)}, true);
  EXPECT_EQ(buffer.size(), 2U);  // the third one replaced one of the identical ones
  EXPECT_EQ(buffer.replaced(), 1U);
}

TEST(DetectionBuffer, MergesThousandsOfDetectionsOfOneCrowdToASparseSet)
{
  // What the filter is good for: a detector reporting a crowd of people for a long time would otherwise remember
  // every report of every person.
  DetectionBuffer buffer;
  size_t added = 0;
  for (int step = 0; step < 100; ++step) {
    std::vector<StoredDetection> message;
    for (int person = 0; person < 20; ++person) {
      // Each person moves by a few centimetres and is detected a tenth of a second later.
      const auto time = static_cast<int64_t>(step) * SECOND / 10;
      message.push_back(makeDetection(time, static_cast<double>(person) + .005 * step,
        100. * person + .3 * step));
    }
    added += message.size();
    buffer.add(std::move(message), true);
  }

  EXPECT_EQ(added, 2000U);
  EXPECT_EQ(buffer.size(), 20U);
}

TEST(DetectionBuffer, KeepsThePreviewOfTheDetectionWhichWasReplaced)
{
  DetectionBuffer buffer;

  auto first = makeDetection(1 * SECOND, 10., 100.);
  buffer.add({first}, true);
  buffer.add({makeDetection(2 * SECOND, 10.1, 110.)}, true);

  // The preview stays the image the detection was first seen in, so that its texture does not have to be created
  // again and again.
  ASSERT_EQ(buffer.size(), 1U);
  EXPECT_EQ(at(buffer, 0).previewStamp.nanoseconds(), 1 * SECOND);
  EXPECT_DOUBLE_EQ(at(buffer, 0).previewRegion.x, 100.);
}

TEST(DetectionBuffer, TakesNewPreviewWhenDetectionMovesAway)
{
  DetectionBuffer buffer;

  // The preview is 40 px wide, so moving by more than a half of it means the detection is not in it anymore.
  buffer.add({makeDetection(1 * SECOND, 10., 100.)}, true);
  ASSERT_EQ(buffer.size(), 1U);
  EXPECT_EQ(at(buffer, 0).previewStamp.nanoseconds(), 1 * SECOND);

  // Moving away is not enough to take a new preview more often than once per second...
  buffer.add({makeDetection(1 * SECOND + SECOND / 2, 10., 140.)}, true);
  EXPECT_EQ(buffer.size(), 1U);
  EXPECT_EQ(at(buffer, 0).previewStamp.nanoseconds(), 1 * SECOND);
  // ...but when it is a while since the preview was taken, the detection which moved takes a new one.
  buffer.add({makeDetection(3 * SECOND, 10., 140.)}, true);
  EXPECT_EQ(buffer.size(), 1U);
  EXPECT_EQ(at(buffer, 0).previewStamp.nanoseconds(), 3 * SECOND);
  EXPECT_DOUBLE_EQ(at(buffer, 0).previewRegion.x, 140.);
}

TEST(DetectionBuffer, DoesNotMergeDetectionsOfDifferentFrames)
{
  DetectionBuffer buffer;

  auto first = makeMessage(1 * SECOND, {"a"});
  buffer.add(first, true);
  auto other = makeMessage(2 * SECOND, {"b"});
  other[0].frameId = "other_frame";  // the same place, but in another frame, which means another place
  buffer.add(other, true);

  EXPECT_EQ(buffer.size(), 2U);
  EXPECT_EQ(buffer.replaced(), 0U);
}

TEST(DetectionBuffer, DropsDetectionsWhichWereNotReportedAgain)
{
  DetectionBuffer buffer;
  withoutDeduplication(buffer);

  buffer.add(makeMessage(1 * SECOND, {"a"}), true);
  buffer.add(makeMessage(9 * SECOND, {"b"}), true);

  EXPECT_EQ(buffer.prune(10 * SECOND, 5., 5.), 1U);
  ASSERT_EQ(buffer.size(), 1U);
  EXPECT_EQ(at(buffer, 0).id, "b");

  // A detection which keeps being reported stays, no matter how old its first report was.
  buffer.add(makeMessage(11 * SECOND, {"b"}), true);
  EXPECT_EQ(buffer.prune(13 * SECOND, 5., 5.), 0U);
  EXPECT_EQ(buffer.size(), 1U);

  // Zero means the detections stay until they are dropped by the limit.
  EXPECT_EQ(buffer.prune(1000 * SECOND, 0., 0.), 0U);
}

TEST(DetectionBuffer, KeepsTheDetectionsInAZoneLonger)
{
  DetectionBuffer buffer;
  withoutDeduplication(buffer);

  auto outside = makeMessage(1 * SECOND, {"out"});
  auto inside = makeMessage(1 * SECOND, {"in"});
  inside[0].inZoneNow = true;
  inside[0].nowZoneIds = {2};
  buffer.add(outside, true);
  buffer.add(inside, true);

  // The detections which are out of any zone are forgotten first, the ones which got into a zone stay.
  EXPECT_EQ(buffer.prune(10 * SECOND, 5., 20.), 1U);
  ASSERT_EQ(buffer.size(), 1U);
  EXPECT_EQ(at(buffer, 0).id, "in");

  // ... unless their own lifetime passed too.
  EXPECT_EQ(buffer.prune(25 * SECOND, 5., 20.), 1U);
  EXPECT_EQ(buffer.size(), 0U);
}

TEST(DetectionBuffer, UsesTheZoneOfTheDetectionForItsLifetime)
{
  DetectionBuffer buffer;
  withoutDeduplication(buffer);

  auto zone = makeMessage(1 * SECOND, {"in", "out", "unknown"});
  zone[0].hasZone = true;  // the membership the detection arrived with, no zones are known now
  zone[0].inZone = true;
  zone[1].hasZone = true;
  zone[1].inZone = false;
  buffer.add(zone, true);

  EXPECT_EQ(buffer.prune(10 * SECOND, 5., 20.), 2U);
  ASSERT_EQ(buffer.size(), 1U);
  EXPECT_EQ(at(buffer, 0).id, "in");
}

TEST(DetectionBuffer, AgesTheStoredDetectionsLikeTheReceivedOnes)
{
  DetectionBuffer buffer;
  withoutDeduplication(buffer);

  // The detections from the storage are old on purpose, but the user wants to see the recent past, not the whole
  // history - a detection which was recorded long ago disappears after the same time as a received one.
  auto recent = makeMessage(100 * SECOND, {"live"});
  auto stored = makeMessage(1 * SECOND, {"stored"});
  stored[0].history = true;
  auto inZone = makeMessage(2 * SECOND, {"stored in a zone"});
  inZone[0].history = true;
  inZone[0].inZoneNow = true;
  buffer.add(stored, true);
  buffer.add(inZone, true);
  buffer.add(recent, true);
  ASSERT_EQ(buffer.size(), 3U);

  // The stored detections are older than both lifetimes from the beginning, the received one is fresh.
  EXPECT_EQ(buffer.prune(100 * SECOND, 5., 20.), 2U);
  ASSERT_EQ(buffer.size(), 1U);
  EXPECT_EQ(at(buffer, 0).id, "live");

  // A detection without any timestamp has no age to compare, so it stays whatever its lifetime is.
  auto timeless = makeMessage(0 * SECOND, {"no stamp"});
  buffer.add(timeless, true);
  ASSERT_EQ(buffer.size(), 2U);
  EXPECT_EQ(buffer.prune(10000 * SECOND, 5., 5.), 1U);
  ASSERT_EQ(buffer.size(), 1U);
  EXPECT_EQ(at(buffer, 0).id, "no stamp");
}

TEST(DetectionBuffer, FindsTheDetectionAgainAfterItsPoseMovesToAnotherFrame)
{
  DetectionBuffer buffer;
  withoutDeduplication(buffer);

  auto arrived = makeMessage(1 * SECOND, {"a"});
  arrived[0].frameId = "camera";  // the pose could not be resolved to the global frame when it arrived
  buffer.add(arrived, true);
  ASSERT_EQ(buffer.size(), 1U);

  // What the display does some time later, when the transform finally comes: it moves the pose to the global frame.
  auto& stored = buffer.detections().front();
  stored.frameId = "map";
  stored.inGlobalFrame = true;
  buffer.reindex();

  // The object reported again is still the detection which is remembered, not a second one next to it.
  buffer.add(makeMessage(2 * SECOND, {"a"}), true);
  EXPECT_EQ(buffer.size(), 1U);
  EXPECT_EQ(buffer.replaced(), 0U);
}

TEST(DetectionBuffer, SeparatesTheStoredDetectionsFromTheReceivedOnes)
{
  DetectionBuffer buffer;
  withoutDeduplication(buffer);

  auto stored = makeMessage(1 * SECOND, {"old"});
  stored[0].history = true;
  buffer.add(stored, true);
  buffer.add(makeMessage(2 * SECOND, {"live"}), true);
  ASSERT_EQ(buffer.size(), 2U);

  // The detections of the newest message replace the previously received ones, but not the stored detections.
  buffer.add(makeMessage(3 * SECOND, {"new"}), false);
  ASSERT_EQ(buffer.size(), 2U);
  EXPECT_TRUE(at(buffer, 0).history);
  EXPECT_EQ(at(buffer, 1).id, "new");

  EXPECT_EQ(buffer.clearHistory(), 1U);
  ASSERT_EQ(buffer.size(), 1U);
  EXPECT_EQ(at(buffer, 0).id, "new");

  buffer.clear();
  EXPECT_EQ(buffer.size(), 0U);
}

TEST(DetectionBuffer, StoredAndReceivedPictureStayApart)
{
  DetectionBuffer buffer;
  // A detection read from the persistent storage and a detection received on the topic are the same object when the
  // recorder and the detector report the same tracker ID - which is what happens when the same mission is replayed.

  auto stored = makeDetection(1 * SECOND, 10., 655., "tracked");
  stored.history = true;
  stored.imagePath = "/tmp/detection.jpg";
  stored.imageRegion = {0., 0., 567., 1026.};  // the whole saved crop, as the display of a stored detection shows it
  stored.previewRegion = stored.imageRegion;
  stored.previewStamp = stored.stamp;
  stored.previewTaken = true;
  buffer.add({stored}, true);
  ASSERT_EQ(buffer.size(), 1U);

  // The same object reported on the topic has its preview cut around its bounding box, exactly like before the stored
  // detection was loaded. Taking the picture of the stored detection over would show a piece of the camera image which
  // has nothing to do with the detection - a crop as big as the whole image, which is what the operator sees as a
  // preview of the whole image instead of the detection.
  buffer.add({makeDetection(2 * SECOND, 10.1, 655., "tracked")}, true);
  ASSERT_EQ(buffer.size(), 1U);
  EXPECT_FALSE(at(buffer, 0).history);
  EXPECT_EQ(at(buffer, 0).previewStamp.nanoseconds(), 2 * SECOND);
  EXPECT_DOUBLE_EQ(at(buffer, 0).previewRegion.x, 655.);
  EXPECT_DOUBLE_EQ(at(buffer, 0).previewRegion.width, 40.);
  EXPECT_DOUBLE_EQ(at(buffer, 0).previewRegion.height, 80.);

  // The other way round: a stored detection which takes the place of a received one keeps the picture saved with it,
  // not the cutout of the camera image the received detection showed.
  auto again = stored;
  again.stamp = rclcpp::Time(3 * SECOND, RCL_ROS_TIME);
  buffer.add({again}, true);
  ASSERT_EQ(buffer.size(), 1U);
  EXPECT_TRUE(at(buffer, 0).history);
  EXPECT_DOUBLE_EQ(at(buffer, 0).previewRegion.width, 567.);
  EXPECT_DOUBLE_EQ(at(buffer, 0).previewRegion.height, 1026.);
}

TEST(DetectionBuffer, EachStoredDetectionShowsItsOwnCrop)
{
  DetectionBuffer buffer;
  buffer.setDeduplication({1., 0., 5.});  // the same object reported again replaces the older record

  auto first = makeDetection(1 * SECOND, 10., 100.);
  first.history = true;
  first.imagePath = "/crops/first.jpg";
  first.imageRegion = {0., 0., 224., 963.};  // the picture saved with the record, shown as a whole
  buffer.add({first}, true);
  ASSERT_EQ(buffer.size(), 1U);
  EXPECT_DOUBLE_EQ(at(buffer, 0).previewRegion.width, 224.);
  EXPECT_DOUBLE_EQ(at(buffer, 0).previewRegion.height, 963.);

  // The same person recorded again a second later replaces the older record - that is the point of the deduplication -
  // but its saved picture is another crop of another size. The place in the older crop means nothing in the newer one,
  // so taking the preview over would show the object somewhere else of the picture or the whole picture of it.
  auto second = makeDetection(2 * SECOND, 10.1, 100.);
  second.history = true;
  second.imagePath = "/crops/second.jpg";
  second.imageRegion = {0., 0., 413., 1143.};
  buffer.add({second}, true);
  ASSERT_EQ(buffer.size(), 1U);
  EXPECT_EQ(at(buffer, 0).imagePath, "/crops/second.jpg");
  EXPECT_DOUBLE_EQ(at(buffer, 0).previewRegion.width, 413.);
  EXPECT_DOUBLE_EQ(at(buffer, 0).previewRegion.height, 1143.);

  // A click in a camera image highlights the stored detection too, but its picture stays the crop saved with it: the
  // clicked image is a picture of the camera as it looks now, and the place in the saved crop means nothing in it.
  EXPECT_FALSE(buffer.retakePreview(at(buffer, 0).serial, clickedPoint("camera", 4 * SECOND, 100., 100.)));
  EXPECT_DOUBLE_EQ(at(buffer, 0).previewRegion.width, 413.);
  EXPECT_EQ(at(buffer, 0).previewStamp.nanoseconds(), 2 * SECOND);
}

TEST(DetectionBuffer, ReportsChangesOfItsContent)
{
  DetectionBuffer buffer;

  const auto initial = buffer.revision();
  buffer.add(makeMessage(1 * SECOND, {"a"}), true);
  EXPECT_NE(buffer.revision(), initial);
  buffer.add({}, true);  // nothing changed
  EXPECT_EQ(buffer.revision(), buffer.revision());

  const auto afterAdd = buffer.revision();
  buffer.clear();
  EXPECT_NE(buffer.revision(), afterAdd);
}

TEST(DetectionBuffer, ReportsTheNewestDetection)
{
  DetectionBuffer buffer;

  EXPECT_EQ(buffer.newestStamp(), 0);
  buffer.add(makeMessage(3 * SECOND, {"a"}), true);
  buffer.add(makeMessage(7 * SECOND, {"b"}), true);
  EXPECT_EQ(buffer.newestStamp(), 7 * SECOND);
}

TEST(Deduplication, IsOnByDefault)
{
  const Deduplication deduplication;
  EXPECT_TRUE(deduplication.enabled());
  EXPECT_GT(deduplication.maxDistance3d, 0.);
}

TEST(DetectionBuffer, SelectsTheDetectionTheUserClickedOn)
{
  DetectionBuffer buffer;
  withoutDeduplication(buffer);
  // The detections of one message are 100 px apart in the image: a at x 0-40, b at x 100-140, both at y 100-180.
  buffer.add(makeMessage(1 * SECOND, {"a", "b"}), true);
  ASSERT_EQ(buffer.size(), 2U);
  const auto a = at(buffer, 0).serial;
  const auto b = at(buffer, 1).serial;

  EXPECT_EQ(buffer.select(clickedPoint("", 1 * SECOND, 20., 140.)), std::vector<uint64_t>{a});
  EXPECT_EQ(buffer.select(clickedPoint("", 1 * SECOND, 120., 140.)), std::vector<uint64_t>{b});
  // A place close next to a bounding box still belongs to it - a person is rarely exactly inside its box.
  EXPECT_EQ(buffer.select(clickedPoint("", 1 * SECOND, 150., 140.)), std::vector<uint64_t>{b});
  // The image alone (a click without a place) makes every detection of that time a candidate.
  EXPECT_EQ(buffer.select(clickedImage("", 1 * SECOND)).size(), 2U);
}

TEST(DetectionBuffer, PrefersTheDetectionUnderTheClick)
{
  DetectionBuffer buffer;
  withoutDeduplication(buffer);
  buffer.add(makeMessage(1 * SECOND, {"a"}), true);
  const auto a = at(buffer, 0).serial;

  // Half a screen away from the only detection: the click cannot be compared to any bounding box, but the clicked
  // image still says when it was taken, and a click which would highlight nothing is worse than a doubtful guess.
  EXPECT_EQ(buffer.select(clickedPoint("", 1 * SECOND, 600., 140.)), (std::vector<uint64_t>{a}));
  auto far = makeDetection(1 * SECOND + SECOND / 2, 1., 600.);  // the detection the clicked place belongs to
  buffer.add({far}, true);
  const auto farSerial = at(buffer, 1).serial;
  EXPECT_EQ(buffer.select(clickedPoint("", 1 * SECOND, 600., 140.)),
    std::vector<uint64_t>{farSerial});
  // The camera of the clicked image is respected even in this fallback - the user cannot have clicked on an object of
  // another camera in it.
  auto other = makeDetection(1 * SECOND, 1., 100.);
  other.cameraFrame = "another_camera";
  buffer.add({other}, true);
  EXPECT_EQ(buffer.select(clickedPoint("third_camera", 1 * SECOND, 300., 400.)),
      (std::vector<uint64_t>{a, farSerial}));
  // The clicked image is from another time than the detection (and the detections do not know their camera, so the
  // frame alone cannot exclude it).
  EXPECT_TRUE(buffer.select(clickedPoint("", 5 * SECOND, 20., 140.)).empty());
  // The default tolerance accepts a detection which is a little older than the clicked image.
  EXPECT_EQ(buffer.select(clickedPoint("", 2 * SECOND, 20., 140.)), std::vector<uint64_t>{a});
  auto exact = clickedPoint("", 2 * SECOND, 20., 140.);
  exact.tolerance = 0.;
  EXPECT_TRUE(buffer.select(exact).empty());
}


TEST(DetectionBuffer, SelectsTheNewestDetectionWhenTheTimeIsUnknown)
{
  DetectionBuffer buffer;
  withoutDeduplication(buffer);
  buffer.add(makeMessage(1 * SECOND, {"old"}), true);
  buffer.add(makeMessage(2 * SECOND, {"new"}), true);
  ASSERT_EQ(buffer.size(), 2U);
  const auto older = at(buffer, 0).serial;
  const auto newer = at(buffer, 1).serial;

  // An image without a stamp says nothing about the time, but the detections still have to be told apart somehow.
  const auto found = buffer.select(clickedImage("", 0));
  ASSERT_EQ(found.size(), 2U);
  EXPECT_EQ(found[0], newer);
  EXPECT_EQ(found[1], older);
}

TEST(DetectionBuffer, PrefersTheSmallestBoundingBoxUnderTheClick)
{
  DetectionBuffer buffer;
  withoutDeduplication(buffer);
  auto carrier = makeDetection(1 * SECOND, 1., 100.);  // x 100-140, y 100-180
  carrier.imageRegion.width = 200.;  // the whole scene, e.g. a group of people detected as one
  carrier.imageRegion.height = 280.;
  auto person = makeDetection(1 * SECOND, 1., 120.);  // x 120-160, y 100-180, inside the other bounding box
  buffer.add({carrier, person}, true);
  ASSERT_EQ(buffer.size(), 2U);
  const auto wide = at(buffer, 0).serial;
  const auto small = at(buffer, 1).serial;

  // Both detections were seen at the clicked place, and the user could only have clicked on the smaller one.
  EXPECT_EQ(buffer.select(clickedPoint("", 1 * SECOND, 130., 140.)),
      (std::vector<uint64_t>{small, wide}));
}

TEST(DetectionBuffer, PrefersTheCameraTheUserClickedIn)
{
  DetectionBuffer buffer;
  withoutDeduplication(buffer);
  auto left = makeDetection(1 * SECOND, 1., 100.);
  left.cameraFrame = "camera_left";
  auto right = makeDetection(1 * SECOND, 1., 100.);
  right.cameraFrame = "camera_right";
  buffer.add({left, right}, true);
  ASSERT_EQ(buffer.size(), 2U);
  const auto leftSerial = at(buffer, 0).serial;
  const auto rightSerial = at(buffer, 1).serial;

  // Two cameras looking at the same place see the same object at the same time, so the clicked camera decides.
  EXPECT_EQ(buffer.select(clickedImage("camera_right", 1 * SECOND)),
      (std::vector<uint64_t>{rightSerial, leftSerial}));
  // A detection which does not know its camera is not penalized - the detections are often reported elsewhere - but a
  // detection from the clicked camera still wins over it.
  auto unknown = makeDetection(1 * SECOND + SECOND / 2, 1., 100.);
  buffer.add({unknown}, true);
  const auto unknownSerial = at(buffer, 2).serial;
  const auto found = buffer.select(clickedImage("camera_right", 1 * SECOND));
  ASSERT_EQ(found.size(), 3U);
  EXPECT_EQ(found[0], rightSerial);
  EXPECT_EQ(found[1], unknownSerial);
  EXPECT_EQ(found[2], leftSerial);
}

TEST(DetectionBuffer, FallsBackToTheImageWhenTheDetectionHasNoPlaceInIt)
{
  DetectionBuffer buffer;
  withoutDeduplication(buffer);
  auto detection = makeDetection(1 * SECOND, 1., 100.);
  detection.cameraFrame = "camera";
  detection.imageRegion = {0., 0., 0., 0.};  // a detector reporting the pose in a frame of its robot
  buffer.add({detection}, true);
  const auto serial = at(buffer, 0).serial;

  // The clicked place cannot be compared to a bounding box the detection does not have, but the clicked image still
  // says the camera and the time, and that is enough to highlight the one detection which fits.
  EXPECT_EQ(buffer.select(clickedPoint("camera", 1 * SECOND, 20., 140.)),
    std::vector<uint64_t>{serial});
  EXPECT_EQ(buffer.select(clickedImage("camera", 1 * SECOND)), std::vector<uint64_t>{serial});
  // A click in an image of another time is not the detection, even when there is nothing else to choose from.
  EXPECT_TRUE(buffer.select(clickedPoint("camera", 9 * SECOND, 20., 140.)).empty());
}

TEST(DetectionBuffer, MatchesTheImageAPreviewWasTakenFrom)
{
  DetectionBuffer buffer;
  buffer.add({makeDetection(1 * SECOND, 1., 100.)}, true);
  ASSERT_EQ(buffer.size(), 1U);
  const auto serial = at(buffer, 0).serial;

  // The preview of a detection which stays shows a newer image than the message the detection arrived in, and that is
  // the image the user sees and can click in.
  auto& stored = buffer.detections().front();
  stored.previewTaken = true;
  stored.previewStamp = rclcpp::Time(2 * SECOND, RCL_ROS_TIME);
  EXPECT_EQ(buffer.select(clickedPoint("", 2 * SECOND, 120., 140.)), std::vector<uint64_t>{serial});
  EXPECT_TRUE(buffer.select(clickedPoint("", 5 * SECOND, 120., 140.)).empty());
}

TEST(DetectionBuffer, ShowsTheImageTheUserClickedIn)
{
  DetectionBuffer buffer;
  withoutDeduplication(buffer);
  buffer.add(makeMessage(1 * SECOND, {"a"}), true);
  const auto serial = at(buffer, 0).serial;
  // The preview sticks to the image the detection was first seen in, which is what keeps the preview of a reported
  // object the same. The user clicked in a newer image, so that one has to be shown instead of the old one.
  ASSERT_EQ(at(buffer, 0).previewStamp.nanoseconds(), 1 * SECOND);

  const auto clicked = clickedPoint("", 3 * SECOND, 30., 140.);  // the bounding box of the detection is at x 0-40
  EXPECT_TRUE(buffer.retakePreview(serial, clicked));
  EXPECT_EQ(at(buffer, 0).previewStamp.nanoseconds(), 3 * SECOND);
  EXPECT_TRUE(at(buffer, 0).previewTaken);
  // The click is on the object, so the preview keeps the cutout of the object: its bounding box with the surroundings
  // the display adds around it, i.e. the same cutout the picture of the object has in the visualization strip.
  EXPECT_DOUBLE_EQ(at(buffer, 0).previewRegion.x, 0.);
  EXPECT_DOUBLE_EQ(at(buffer, 0).previewRegion.y, 100.);
  EXPECT_DOUBLE_EQ(at(buffer, 0).previewRegion.width, 40.);
  EXPECT_DOUBLE_EQ(at(buffer, 0).previewRegion.height, 80.);
  EXPECT_EQ(buffer.select(clicked), std::vector<uint64_t>{serial});

  // The user pointed beside the box, at another object of the picture: the place of the click is then the only place
  // known to be in the clicked image, so the shown part of it is centred on it.
  EXPECT_TRUE(buffer.retakePreview(serial, clickedPoint("", 3 * SECOND, 200., 300.)));
  EXPECT_DOUBLE_EQ(at(buffer, 0).previewRegion.x, 180.);
  EXPECT_DOUBLE_EQ(at(buffer, 0).previewRegion.y, 260.);
  EXPECT_DOUBLE_EQ(at(buffer, 0).previewRegion.width, 40.);
  EXPECT_DOUBLE_EQ(at(buffer, 0).previewRegion.height, 80.);

  // A click without a place says only which image; the bounding box of the detection is the only place to show then.
  EXPECT_TRUE(buffer.retakePreview(serial, clickedImage("", 4 * SECOND)));
  EXPECT_DOUBLE_EQ(at(buffer, 0).previewRegion.x, 0.);
  EXPECT_DOUBLE_EQ(at(buffer, 0).previewRegion.y, 100.);

  // An image without a stamp cannot be shown, and a detection which is not remembered cannot be asked for.
  EXPECT_FALSE(buffer.retakePreview(serial, clickedImage("", 0)));
  EXPECT_FALSE(buffer.retakePreview(serial + 100, clickedImage("", 5 * SECOND)));
  EXPECT_EQ(at(buffer, 0).previewStamp.nanoseconds(), 4 * SECOND);
}
