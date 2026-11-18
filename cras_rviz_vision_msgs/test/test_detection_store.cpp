// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Tests of reading the persistent detection storage.
 * \author Czech Technical University in Prague
 */

#include "cras_rviz_vision_msgs/detection_store.hpp"

#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace cras_rviz_vision_msgs {

namespace {

const char HEADER[] = "id,stamp,stamp_unix_ns,frame_id,class_name,probability,det_id,bbox_x,bbox_y,bbox_w,"
                      "bbox_h,x,y,z,qx,qy,qz,qw,ecef_frame,ecef_x,ecef_y,ecef_z,ecef_qx,ecef_qy,ecef_qz,"
                      "ecef_qw,latitude,longitude,altitude,zone_frame,zone_x,zone_y,zone_z,in_zone,zone_ids,"
                      "image_path,image_w,image_h";

//! One row of the CSV file, with the columns the test does not care about left empty.
std::string row(const std::string& id, int64_t stampNs, const std::string& className, const std::string& inZone,
    const std::string& zoneIds = "", const std::string& imagePath = "", double zoneX = 1.5, double zoneY = 2.5,
    const std::string& ecef = "") {
  // the global position (and the geodetic coordinates derived from it) is stored only when known
  std::string global = ",,,,,,,,,,,";
  if (!ecef.empty()) {
    global = ",FP_ECEF," + ecef + ",0,0,0,1,50.0875,14.4213,200.";
  }
  return id + ",2026-11-17T12:00:00+00:00," + std::to_string(stampNs) + ",camera_frame," + className +
  ",0.75,7,100,200,40,80,1,2,3,0,0,0,1" + global + ",map," + std::to_string(zoneX) + "," +
  std::to_string(zoneY) + ",0.5," + inZone + "," + zoneIds + "," + imagePath + ",40,20";
}

void writeFile(const std::string& path, const std::vector<std::string>& lines) {
  std::ofstream stream(path);
  ASSERT_TRUE(stream.is_open());
  stream << HEADER << '\n';
  for (const auto& line : lines) {
    stream << line << '\n';
  }
}

std::string storageDir(const std::string& name) {
  return testing::TempDir() + "cras_rviz_vision_msgs_" + name;
}

//! Timestamp of the given moment in UTC, in nanoseconds since Unix epoch.
int64_t utcNs(int year, unsigned month, unsigned day, unsigned hour, unsigned minute = 0) {
  std::tm moment{};
  moment.tm_year = year - 1900;
  moment.tm_mon = static_cast<int>(month) - 1;
  moment.tm_mday = static_cast<int>(day);
  moment.tm_hour = static_cast<int>(hour);
  moment.tm_min = static_cast<int>(minute);
  moment.tm_isdst = 0;
  return static_cast<int64_t>(timegm(&moment)) * 1000000000LL;
}

}  // namespace

TEST(DetectionStore, defaultRootDirFollowsRosHome) {
  const char* previous = std::getenv("ROS_HOME");
  setenv("ROS_HOME", "/tmp/ros_home_test", 1);
  EXPECT_EQ(DetectionStore::defaultRootDir(), "/tmp/ros_home_test/g4s_mission");
  if (previous) {
    setenv("ROS_HOME", previous, 1);
  } else {
    unsetenv("ROS_HOME");
  }
  const char* home = std::getenv("HOME");
  if (home && !std::getenv("ROS_HOME")) {
    EXPECT_EQ(DetectionStore::defaultRootDir(), std::string(home) + "/.ros/g4s_mission");
  }
}

TEST(DetectionStore, expandUser) {
  const char* home = std::getenv("HOME");
  if (!home) {
    GTEST_SKIP() << "HOME is not set";
  }
  EXPECT_EQ(DetectionStore::expandUser("~/detections"), std::string(home) + "/detections");
  EXPECT_EQ(DetectionStore::expandUser("~"), std::string(home));
  EXPECT_EQ(DetectionStore::expandUser("/tmp/x"), "/tmp/x");
  EXPECT_EQ(DetectionStore::expandUser("  "), "");
}

TEST(DetectionStore, splitLine) {
  EXPECT_EQ(DetectionStore::splitLine("a,b,c"), (std::vector<std::string>{"a", "b", "c"}));
  EXPECT_EQ(DetectionStore::splitLine("a,,c"), (std::vector<std::string>{"a", "", "c"}));
  EXPECT_EQ(DetectionStore::splitLine("a,\"with, comma\",c"),
      (std::vector<std::string>{"a", "with, comma", "c"}));
  EXPECT_EQ(DetectionStore::splitLine("a,\"say \"\"hi\"\"\",c"),
      (std::vector<std::string>{"a", "say \"hi\"", "c"}));
  EXPECT_EQ(DetectionStore::splitLine(" a , b "), (std::vector<std::string>{"a", "b"}));
  // an unfinished line (still being written by the recorder) must not be interpreted
  EXPECT_TRUE(DetectionStore::splitLine("a,\"unfinished").empty());
}

TEST(DetectionStore, parseRow) {
  std::map<std::string, std::string> values;
  const auto fields = DetectionStore::splitLine(HEADER);
  const auto data = DetectionStore::splitLine(row("id1", 1000000000, "person", "in", "3;1", "images/a.jpg"));
  ASSERT_EQ(fields.size(), data.size());
  for (size_t i = 0; i < fields.size(); ++i) {
    values[fields[i]] = data[i];
  }

  StoredRecord record;
  ASSERT_TRUE(DetectionStore::parseRow(values, "/storage", record));
  EXPECT_EQ(record.id, "id1");
  EXPECT_EQ(record.stampNs, 1000000000);
  EXPECT_EQ(record.className, "person");
  EXPECT_DOUBLE_EQ(record.probability, .75);
  EXPECT_EQ(record.detectionId, "7");
  EXPECT_TRUE(record.hasPose);
  EXPECT_DOUBLE_EQ(record.y, 2.);
  EXPECT_TRUE(record.inZone());
  EXPECT_EQ(record.zoneIds, (std::vector<int>{3, 1}));
  EXPECT_TRUE(record.hasZonePosition);
  EXPECT_DOUBLE_EQ(record.zoneX, 1.5);
  EXPECT_EQ(record.imagePath, "/storage/images/a.jpg");
  EXPECT_EQ(record.imageWidth, 40U);
  EXPECT_EQ(record.imageHeight, 20U);

  double position[3];
  double orientation[4];
  record.getDisplayPose(position, orientation);
  EXPECT_DOUBLE_EQ(position[0], 1.5);
  EXPECT_DOUBLE_EQ(orientation[3], 1.);
  EXPECT_EQ(record.displayFrame(), "map");

  // the values the recorder could not know stay unused
  values["zone_x"] = "";
  values["zone_z"] = "";
  values["in_zone"] = "unknown";
  values["zone_ids"] = "";
  values["image_path"] = "";
  values["probability"] = "";
  ASSERT_TRUE(DetectionStore::parseRow(values, "/storage", record));
  EXPECT_FALSE(record.hasZonePosition);
  EXPECT_TRUE(record.hasPose);
  EXPECT_EQ(record.displayFrame(), "camera_frame");
  EXPECT_FALSE(record.inZone());
  EXPECT_EQ(record.zoneStatus, "unknown");
  EXPECT_TRUE(record.imagePath.empty());
  EXPECT_DOUBLE_EQ(record.probability, 1.);

  // a row without any stamp is useless
  values["stamp_unix_ns"] = "";
  values["stamp"] = "not a time";
  EXPECT_FALSE(DetectionStore::parseRow(values, "/storage", record));

  // the ISO stamp is a valid fallback when the numeric one is missing
  values["stamp"] = "2026-11-17T12:00:00+00:00";
  ASSERT_TRUE(DetectionStore::parseRow(values, "/storage", record));
  EXPECT_EQ(record.stampNs, utcNs(2026, 11, 17, 12));
  values["stamp"] = "2026-11-17T12:00:00.5Z";
  ASSERT_TRUE(DetectionStore::parseRow(values, "/storage", record));
  EXPECT_EQ(record.stampNs, utcNs(2026, 11, 17, 12) + 500000000);
  // a stamp in another timezone stands for the same moment in UTC
  values["stamp"] = "2026-11-17T13:15:00+01:30";
  ASSERT_TRUE(DetectionStore::parseRow(values, "/storage", record));
  EXPECT_EQ(record.stampNs, utcNs(2026, 11, 17, 11, 45));
  values["stamp"] = "2026-11-17T11:45:00-01:00";
  ASSERT_TRUE(DetectionStore::parseRow(values, "/storage", record));
  EXPECT_EQ(record.stampNs, utcNs(2026, 11, 17, 12, 45));
  // the row of the test has no global position, so there is nothing to prefer over the recorded frame
  EXPECT_FALSE(record.hasEcefPosition);
  EXPECT_TRUE(record.ecefFrame.empty());
  record.preferGlobalFrame = true;
  EXPECT_FALSE(record.usesGlobalFrame());
  EXPECT_EQ(record.displayFrame(), "camera_frame");
}

TEST(DetectionStore, parseGlobalPosition) {
  std::map<std::string, std::string> values;
  const auto fields = DetectionStore::splitLine(HEADER);
  const auto data = DetectionStore::splitLine(row("id1", 1000000000, "person", "in", "3", "", 1.5, 2.5,
      "3971326.9,1021237.3,4869192.8"));
  ASSERT_EQ(fields.size(), data.size());
  for (size_t i = 0; i < fields.size(); ++i) {
    values[fields[i]] = data[i];
  }

  StoredRecord record;
  ASSERT_TRUE(DetectionStore::parseRow(values, "/storage", record));
  EXPECT_TRUE(record.hasEcefPosition);
  EXPECT_EQ(record.ecefFrame, "FP_ECEF");
  EXPECT_DOUBLE_EQ(record.ecefX, 3971326.9);
  EXPECT_DOUBLE_EQ(record.ecefZ, 4869192.8);
  EXPECT_DOUBLE_EQ(record.ecefQw, 1.);
  EXPECT_TRUE(record.hasZonePosition);

  double position[3];
  double orientation[4];
  // the frame the detection survives between missions in is used once the caller asks for it
  record.preferGlobalFrame = true;
  ASSERT_TRUE(record.usesGlobalFrame());
  record.getDisplayPose(position, orientation);
  EXPECT_DOUBLE_EQ(position[0], 3971326.9);
  EXPECT_DOUBLE_EQ(position[2], 4869192.8);
  EXPECT_DOUBLE_EQ(orientation[3], 1.);
  EXPECT_EQ(record.displayFrame(), "FP_ECEF");

  // without the global position, the detection stays in the frame of the mission it was recorded in
  record.preferGlobalFrame = false;
  record.getDisplayPose(position, orientation);
  EXPECT_DOUBLE_EQ(position[0], 1.5);
  EXPECT_EQ(record.displayFrame(), "map");
}

TEST(DetectionStore, csvFilesAreSelectedByDay) {
  const auto root = storageDir("files");
  std::filesystem::remove_all(root);
  std::filesystem::create_directories(root);
  for (const auto& name : {"detections_2026-11-16.csv", "detections_2026-11-17.csv", "detections_2026-11-18.csv",
         "detections_today.csv", "other_2026-11-17.csv"})
  {
    std::ofstream(root + '/' + name) << HEADER << '\n';
  }

  const auto all = DetectionStore::csvFiles(root, 0, std::numeric_limits<int64_t>::max());
  ASSERT_EQ(all.size(), 4U);  // the file with an undecodable day is rather read than skipped
  EXPECT_EQ(all.front(), root + "/detections_2026-11-16.csv");

  // midnight of 2026-11-17 in the local timezone of the machine
  std::tm moment{};
  moment.tm_year = 126;
  moment.tm_mon = 10;
  moment.tm_mday = 17;
  moment.tm_isdst = -1;
  const int64_t noon = (static_cast<int64_t>(std::mktime(&moment)) + 12 * 3600) * 1000000000LL;
  const auto single = DetectionStore::csvFiles(root, noon, noon);
  // detections_today.csv cannot be dated, so it is read whenever anything else is
  ASSERT_EQ(single.size(), 2U);
  EXPECT_EQ(single[0], root + "/detections_2026-11-17.csv");
  EXPECT_EQ(single[1], root + "/detections_today.csv");

  EXPECT_TRUE(DetectionStore::csvFiles(root + "/does_not_exist", 0,
      std::numeric_limits<int64_t>::max()).empty());
  std::filesystem::remove_all(root);
}

TEST(DetectionStore, loadFiltersAndSorts) {
  const auto root = storageDir("load");
  std::filesystem::remove_all(root);
  std::filesystem::create_directories(root);
  const int64_t base = utcNs(2026, 11, 17, 12);
  writeFile(root + "/detections_2026-11-17.csv", {
      row("a", base + 2000000000, "bicycle", "out"),
      row("b", base + 1000000000, "person", "in", "2", "images/2026-11-17/person/b.jpg"),
      row("c", base + 3000000000, "person", "unknown"),
      "a,short,line",
  });

  std::string error;
  size_t skipped = 0;
  StorageQuery query;
  query.rootDir = root;
  auto records = DetectionStore::load(query, error, &skipped);
  EXPECT_TRUE(error.empty()) << error;
  EXPECT_EQ(skipped, 1U);
  ASSERT_EQ(records.size(), 3U);
  // the records are sorted by their stamp
  EXPECT_EQ(records[0].id, "b");
  EXPECT_EQ(records[1].id, "a");
  EXPECT_EQ(records[2].id, "c");
  EXPECT_EQ(records[0].imagePath, root + "/images/2026-11-17/person/b.jpg");

  query.classes = {"person"};
  records = DetectionStore::load(query, error);
  ASSERT_EQ(records.size(), 2U);

  query.classes.clear();
  query.zoneFilter = 1;
  records = DetectionStore::load(query, error);
  ASSERT_EQ(records.size(), 1U);
  EXPECT_EQ(records[0].id, "b");

  query.zoneFilter = -1;
  records = DetectionStore::load(query, error);
  ASSERT_EQ(records.size(), 2U);

  query.zoneFilter = 0;
  query.limit = 2;
  records = DetectionStore::load(query, error);
  ASSERT_EQ(records.size(), 2U);
  EXPECT_EQ(records[0].id, "a");  // the newest records are kept
  EXPECT_EQ(records[1].id, "c");

  query.limit = 0;
  query.startNs = base + 2500000000;
  records = DetectionStore::load(query, error);
  ASSERT_EQ(records.size(), 1U);
  EXPECT_EQ(records[0].id, "c");

  // a period which is not covered by any of the day files is reported as an empty result
  query.startNs = 0;
  query.endNs = base - 48 * 3600 * 1000000000LL;
  records = DetectionStore::load(query, error);
  EXPECT_TRUE(records.empty());
  EXPECT_FALSE(error.empty());

  // a period with a matching file, but without any stored detection, is not an error
  query.endNs = base - 3600 * 1000000000LL;
  records = DetectionStore::load(query, error);
  EXPECT_TRUE(records.empty());
  EXPECT_TRUE(error.empty()) << error;

  query.rootDir = root + "/does_not_exist";
  records = DetectionStore::load(query, error);
  EXPECT_TRUE(records.empty());
  EXPECT_NE(error.find("does not exist"), std::string::npos) << error;
  std::filesystem::remove_all(root);
}

}  // namespace cras_rviz_vision_msgs
