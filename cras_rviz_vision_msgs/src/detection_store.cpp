// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Implementation of the reading of the persistent detection storage.
 * \author Czech Technical University in Prague
 */

#include "cras_rviz_vision_msgs/detection_store.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace cras_rviz_vision_msgs {

namespace {

//! Name of the folder of the storage inside $ROS_HOME.
constexpr const char* const STORAGE_FOLDER = "g4s_mission";
//! Prefix and suffix of the CSV file of one day.
constexpr const char* const CSV_PREFIX = "detections_";
constexpr const char* const CSV_SUFFIX = ".csv";
//! Nanoseconds in one second.
constexpr int64_t NS_PER_SEC = 1000000000LL;

//! Replaces everything but letters, digits and dashes, so that the value can be used as a file or folder name.
std::string sanitize(const std::string& name) {
  std::string clean;
  for (const auto character : name) {
    const bool letter = (character >= 'a' && character <= 'z') || (character >= 'A' && character <= 'Z') ||
                        (character >= '0' && character <= '9');
    clean += letter || character == '-' || character == '_' ? character : '_';
  }
  return clean;
}

std::string trimmed(const std::string& value) {
  const auto begin = value.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return {};
  }
  const auto end = value.find_last_not_of(" \t\r\n");
  return value.substr(begin, end - begin + 1);
}

bool toDouble(const std::string& value, double& result) {
  const auto text = trimmed(value);
  if (text.empty()) {
    return false;
  }
  try {
    size_t used = 0;
    const double parsed = std::stod(text, &used);
    if (used != text.size() || !std::isfinite(parsed)) {
      return false;
    }
    result = parsed;
    return true;
  } catch (const std::exception&) {
    return false;
  }
}

bool toInt64(const std::string& value, int64_t& result) {
  double parsed = 0.;
  if (!toDouble(value, parsed)) {
    return false;
  }
  result = static_cast<int64_t>(parsed);
  return true;
}

//! Number of days between 1970-01-01 and the given civil date (the algorithm of Howard Hinnant).
int64_t daysFromCivil(int64_t year, unsigned month, unsigned day) {
  year -= month <= 2;
  const int64_t era = (year >= 0 ? year : year - 399) / 400;
  const unsigned yearOfEra = static_cast<unsigned>(year - era * 400);
  const unsigned dayOfYear = (153 * (month + (month > 2 ? -3u : 9u)) + 2) / 5 + day - 1;
  const unsigned dayOfEra = yearOfEra * 365 + yearOfEra / 4 - yearOfEra / 100 + dayOfYear;
  return era * 146097 + static_cast<int64_t>(dayOfEra) - 719468;
}


/**
 * Parses the ISO 8601 stamps written by the detection recorder, i.e. YYYY-MM-DDTHH:MM:SS with an optional fraction of
 * a second and an optional timezone offset (\c Z or [+-]HH:MM, and a missing offset means the local timezone).
 */
bool parseIso8601(const std::string& value, int64_t& stampNs) {
  const auto text = trimmed(value);
  int year = 0;
  unsigned month = 0;
  unsigned day = 0;
  unsigned hour = 0;
  unsigned minute = 0;
  unsigned second = 0;
  char separator = 0;
  int used = 0;
  if (std::sscanf(text.c_str(), "%d-%u-%u%c%u:%u:%u%n", &year, &month, &day, &separator, &hour, &minute, &second,
      &used) != 7 || used <= 0)
  {
    return false;
  }
  if (separator != 'T' && separator != 't' && separator != ' ') {
    return false;
  }
  if (month < 1 || month > 12 || day < 1 || day > 31 || hour > 24 || minute > 59 || second > 60) {
    return false;
  }

  const std::string rest = text.substr(static_cast<size_t>(used));
  double fraction = 0.;
  size_t position = 0;
  if (position < rest.size() && (rest[position] == '.' || rest[position] == ',')) {
    const auto digits = rest.substr(position + 1);
    const auto length = digits.find_first_not_of("0123456789");
    const auto number = digits.substr(0, length == std::string::npos ? digits.size() : length);
    if (!number.empty()) {
      fraction = std::stod("0." + number);
    }
    position += 1 + number.size();
  }

  const std::string zone = trimmed(rest.substr(std::min(position, rest.size())));
  if (zone.empty()) {
    // no timezone in the stamp: it was written in the local timezone of the machine that recorded it
    std::tm local{};
    local.tm_year = year - 1900;
    local.tm_mon = static_cast<int>(month) - 1;
    local.tm_mday = static_cast<int>(day);
    local.tm_hour = static_cast<int>(hour);
    local.tm_min = static_cast<int>(minute);
    local.tm_sec = static_cast<int>(second);
    local.tm_isdst = -1;
    const auto seconds = std::mktime(&local);
    if (seconds == static_cast<std::time_t>(-1)) {
      return false;
    }
    stampNs = static_cast<int64_t>(seconds) * NS_PER_SEC + static_cast<int64_t>(fraction * 1e9);
    return true;
  }

  int offsetSeconds = 0;
  if (zone != "Z" && zone != "z") {
    int zoneHour = 0;
    int zoneMinute = 0;
    char sign = 0;
    if (std::sscanf(zone.c_str(), "%c%d:%d", &sign, &zoneHour, &zoneMinute) < 2) {
      return false;
    }
    offsetSeconds = 3600 * zoneHour + 60 * zoneMinute;
    if (sign == '-') {
      offsetSeconds = -offsetSeconds;
    }
  }
  const auto days = daysFromCivil(year, month, day);
  const int64_t seconds = days * 86400 + 3600 * static_cast<int64_t>(hour) + 60 * static_cast<int64_t>(minute) +
                          static_cast<int64_t>(second) - offsetSeconds;
  if (seconds < 0) {
    return false;
  }
  stampNs = seconds * NS_PER_SEC + static_cast<int64_t>(fraction * 1e9);
  return true;
}

//! First and last second of the given YYYY-MM-DD day (local time), or false if the name cannot be understood.
bool dayRange(const std::string& name, int64_t& first, int64_t& last) {
  int year = 0;
  int month = 0;
  int dayOfMonth = 0;
  char tail = 0;
  if (std::sscanf(name.c_str(), "%d-%d-%d%c", &year, &month, &dayOfMonth, &tail) != 3) {
    return false;
  }
  std::tm moment{};
  moment.tm_year = year - 1900;
  moment.tm_mon = month - 1;
  moment.tm_mday = dayOfMonth;
  moment.tm_isdst = -1;
  const auto seconds = std::mktime(&moment);
  if (seconds == static_cast<std::time_t>(-1)) {
    return false;
  }
  first = static_cast<int64_t>(seconds);
  last = first + 24 * 3600;
  std::tm next{};
  next.tm_year = moment.tm_year;
  next.tm_mon = moment.tm_mon;
  next.tm_mday = moment.tm_mday + 1;
  next.tm_isdst = -1;
  const auto nextSeconds = std::mktime(&next);
  if (nextSeconds != static_cast<std::time_t>(-1)) {
    last = static_cast<int64_t>(nextSeconds);  // handles the daylight saving shifts
  }
  return true;
}

std::string homeDir() {
  const char* home = std::getenv("HOME");
  return home && *home ? std::string(home) : std::string("~");
}

//! Joins the root of the storage with the stored (possibly absolute) path of a detection image.
std::string join(const std::string& root, const std::string& relative) {
  if (relative.empty()) {
    return {};
  }
  if (relative.front() == '/' || (relative.size() > 1 && relative[1] == ':')) {
    return relative;
  }
  if (root.empty()) {
    return relative;
  }
  return root.back() == '/' ? root + relative : root + '/' + relative;
}

}  // namespace

void StoredRecord::getDisplayPose(double position[3], double orientation[4]) const {
  if (usesGlobalFrame()) {
    position[0] = ecefX;
    position[1] = ecefY;
    position[2] = ecefZ;
    orientation[0] = ecefQx;
    orientation[1] = ecefQy;
    orientation[2] = ecefQz;
    orientation[3] = ecefQw;
    return;
  }
  if (hasZonePosition) {
    // The marker is a box, so its orientation does not matter and the identity quaternion is good enough.
    position[0] = zoneX;
    position[1] = zoneY;
    position[2] = zoneZ;
    orientation[0] = 0.;
    orientation[1] = 0.;
    orientation[2] = 0.;
    orientation[3] = 1.;
    return;
  }
  position[0] = x;
  position[1] = y;
  position[2] = z;
  orientation[0] = qx;
  orientation[1] = qy;
  orientation[2] = qz;
  orientation[3] = qw;
}

std::string DetectionStore::defaultRootDir() {
  const char* rosHome = std::getenv("ROS_HOME");
  std::string root = rosHome && *rosHome ? std::string(rosHome) : homeDir() + "/.ros";
  while (root.size() > 1 && root.back() == '/') {
    root.pop_back();
  }
  return join(root, STORAGE_FOLDER);
}

std::string DetectionStore::expandUser(const std::string& path) {
  const auto text = trimmed(path);
  if (text == "~") {
    return homeDir();
  }
  if (text.size() > 1 && text.front() == '~' && text[1] == '/') {
    return join(homeDir(), text.substr(2));
  }
  return text;
}

std::vector<std::string> DetectionStore::splitLine(const std::string& line) {
  std::vector<std::string> fields;
  std::string field;
  bool quoted = false;
  bool hadQuotes = false;
  for (size_t i = 0; i < line.size(); ++i) {
    const char character = line[i];
    if (quoted) {
      if (character != '"') {
        field += character;
      } else if (i + 1 < line.size() && line[i + 1] == '"') {
        field += '"';  // a doubled quote inside a quoted field
        ++i;
      } else {
        quoted = false;
      }
      continue;
    }
    if (character == '"' && field.empty()) {
      quoted = true;
      hadQuotes = true;
    } else if (character == ',') {
      fields.push_back(hadQuotes ? field : trimmed(field));
      field.clear();
      hadQuotes = false;
    } else {
      field += character;
    }
  }
  if (quoted) {
    return {};  // an unterminated quote means the line is still being written
  }
  fields.push_back(hadQuotes ? field : trimmed(field));
  return fields;
}

std::vector<std::string> DetectionStore::csvFiles(const std::string& rootDir, int64_t startNs, int64_t endNs) {
  std::vector<std::string> files;
  std::error_code error;
  if (rootDir.empty() || !std::filesystem::is_directory(rootDir, error)) {
    return files;
  }
  const std::string prefix = CSV_PREFIX;
  const std::string suffix = CSV_SUFFIX;
  for (const auto& entry : std::filesystem::directory_iterator(rootDir, error)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    const auto name = entry.path().filename().string();
    if (name.size() <= prefix.size() + suffix.size() || name.compare(0, prefix.size(), prefix) != 0 ||
        name.compare(name.size() - suffix.size(), suffix.size(), suffix) != 0)
    {
      continue;
    }
    const std::string day = name.substr(prefix.size(), name.size() - prefix.size() - suffix.size());
    int64_t first = 0;
    int64_t last = 0;
    if (dayRange(day, first, last) && (first * NS_PER_SEC > endNs || (last - 1) * NS_PER_SEC < startNs)) {
      continue;  // the whole day is outside of the queried period
    }
    files.push_back(entry.path().string());
  }
  std::sort(files.begin(), files.end());
  return files;
}

std::vector<std::string> DetectionStore::listFiles(const std::string& dir) {
  std::vector<std::string> names;
  std::error_code error;
  if (dir.empty() || !std::filesystem::is_directory(dir, error)) {
    return names;
  }
  for (const auto& entry : std::filesystem::directory_iterator(dir, error)) {
    const auto name = entry.path().filename().string();
    if (!name.empty() && name.front() != '.') {
      names.push_back(name);
    }
  }
  std::sort(names.begin(), names.end());
  return names;
}

const std::vector<std::string>& DetectionStore::imageExtensions() {
  static const std::vector<std::string> extensions {".jpg", ".jpeg", ".png"};
  return extensions;
}

bool DetectionStore::parseRow(const std::map<std::string, std::string>& row, const std::string& rootDir,
    StoredRecord& record) {
  const auto value = [&row](const char* column) -> std::string {
                       const auto found = row.find(column);
                       return found == row.end() ? std::string() : found->second;
                     };

  record = StoredRecord();
  int64_t stamp = 0;
  if (!toInt64(value("stamp_unix_ns"), stamp) && !parseIso8601(value("stamp"), stamp)) {
    return false;  // a row without a usable stamp cannot be placed in time
  }
  record.stampNs = stamp;
  record.id = value("id");
  record.className = value("class_name");
  record.detectionId = value("det_id");
  record.frameId = value("frame_id");
  record.zoneFrame = value("zone_frame");
  record.zoneStatus = value("in_zone");
  record.imagePath = join(rootDir, value("image_path"));
  // The frame of the camera which took the image tells the detections coming from several cameras apart; the old
  // storage files do not have it and their detections are then not told apart.
  record.imageFrame = value("image_frame");
  record.cameraFrame = value("camera_frame");

  double number = 0.;
  if (toDouble(value("probability"), number)) {
    record.probability = number;
  }
  record.hasPose = toDouble(value("x"), record.x) && toDouble(value("y"), record.y) &&
                   toDouble(value("z"), record.z);
  if (record.hasPose && toDouble(value("qw"), record.qw)) {
    toDouble(value("qx"), record.qx);
    toDouble(value("qy"), record.qy);
    toDouble(value("qz"), record.qz);
  }
  // the position in the Earth-centred frame is the one which survives between missions
  record.ecefFrame = value("ecef_frame");
  record.hasEcefPosition = toDouble(value("ecef_x"), record.ecefX) &&
                           toDouble(value("ecef_y"), record.ecefY) &&
                           toDouble(value("ecef_z"), record.ecefZ);
  if (record.hasEcefPosition) {
    if (!toDouble(value("ecef_qw"), record.ecefQw)) {
      record.ecefQw = 1.;
    }
    toDouble(value("ecef_qx"), record.ecefQx);
    toDouble(value("ecef_qy"), record.ecefQy);
    toDouble(value("ecef_qz"), record.ecefQz);
  }
  record.hasZonePosition = toDouble(value("zone_x"), record.zoneX) && toDouble(value("zone_y"), record.zoneY);
  if (record.hasZonePosition) {
    if (!toDouble(value("zone_z"), record.zoneZ)) {
      record.zoneZ = 0.;
    }
    if (record.zoneFrame.empty()) {
      record.zoneFrame = record.frameId;
    }
  }
  record.hasBbox = toDouble(value("bbox_x"), record.bboxX) && toDouble(value("bbox_y"), record.bboxY) &&
                   toDouble(value("bbox_w"), record.bboxW) && toDouble(value("bbox_h"), record.bboxH);
  std::stringstream zones{value("zone_ids")};
  std::string part;
  while (std::getline(zones, part, ';')) {
    int64_t index = -1;
    if (toInt64(part, index) && index >= 0) {
      record.zoneIds.push_back(static_cast<int>(index));
    }
  }
  int64_t size = 0;
  if (toInt64(value("image_w"), size) && size > 0) {
    record.imageWidth = static_cast<unsigned>(size);
  }
  if (toInt64(value("image_h"), size) && size > 0) {
    record.imageHeight = static_cast<unsigned>(size);
  }
  return true;
}

std::vector<StoredRecord> DetectionStore::load(const StorageQuery& query, std::string& error, size_t* skipped) {
  if (skipped) {
    *skipped = 0;
  }
  error.clear();
  const auto rootDir = query.rootDir.empty() ? defaultRootDir() : expandUser(query.rootDir);
  std::vector<StoredRecord> records;
  std::error_code code;
  if (!std::filesystem::is_directory(rootDir, code)) {
    error = "The detection storage " + rootDir + " does not exist.";
    return records;
  }
  const auto files = csvFiles(rootDir, query.startNs, query.endNs);
  if (files.empty()) {
    error = "No detection file for the selected period in " + rootDir + '.';
    return records;
  }

  for (const auto& file : files) {
    std::ifstream stream(file);
    if (!stream) {
      if (error.empty()) {
        error = "Cannot read " + file + '.';
      }
      continue;
    }
    std::vector<std::string> columns;
    std::string line;
    while (std::getline(stream, line)) {
      if (trimmed(line).empty()) {
        continue;
      }
      auto fields = splitLine(line);
      if (columns.empty()) {
        columns = std::move(fields);
        continue;
      }
      StoredRecord record;
      std::map<std::string, std::string> row;
      if (fields.size() != columns.size() || fields.empty()) {
        if (skipped) {
          ++(*skipped);  // a malformed line, or a line which is still being written
        }
        continue;
      }
      for (size_t i = 0; i < columns.size(); ++i) {
        row[columns[i]] = fields[i];
      }
      if (!parseRow(row, rootDir, record)) {
        if (skipped) {
          ++(*skipped);
        }
        continue;
      }
      if (record.stampNs < query.startNs || record.stampNs > query.endNs) {
        continue;
      }
      if (!query.classes.empty() &&
          std::find(query.classes.begin(), query.classes.end(), record.className) == query.classes.end())
      {
        continue;
      }
      if (!query.cameras.empty() &&
          std::find(query.cameras.begin(), query.cameras.end(), record.cameraFrame) == query.cameras.end())
      {
        continue;  // a detection of another camera (or of an unknown one) is never one of the asked for cameras
      }
      if (query.zoneFilter > 0 && !record.inZone()) {
        continue;
      }
      if (query.zoneFilter < 0 && record.inZone()) {
        continue;
      }
      records.push_back(std::move(record));
    }
  }

  std::sort(records.begin(), records.end(), [](const StoredRecord& left, const StoredRecord& right) {
        return left.stampNs < right.stampNs;
  });
  if (query.limit > 0 && records.size() > query.limit) {
    records.erase(records.begin(), records.end() - static_cast<std::ptrdiff_t>(query.limit));
  }
  return records;
}

}  // namespace cras_rviz_vision_msgs
