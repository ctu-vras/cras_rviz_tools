// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Reading of the detections stored in the persistent detection storage.
 * \author Czech Technical University in Prague
 */

#pragma once

#include <cstdint>
#include <limits>
#include <map>
#include <string>
#include <vector>

namespace cras_rviz_vision_msgs {

/**
 * \brief One detection read from the detection storage.
 *
 * The values correspond to the columns of the CSV files written by the detection recorder of the g4s_mission package
 * (see g4s_mission/detection_storage.py). A record which does not have some of the information (an old CSV file, a
 * column left empty because the information was not available when the detection was recorded) is represented by the
 * corresponding default value, so that the caller can decide what to do with it.
 */
struct StoredRecord {
  std::string id;  //!< Unique identifier of the record.
  int64_t stampNs {0};  //!< Detection stamp (nanoseconds since Unix epoch).
  std::string className;  //!< Name of the detected class.
  double probability {1.};  //!< Probability of the detected class.
  std::string detectionId;  //!< ID of the detection in the message it arrived in.
  std::string frameId;  //!< Frame of the pose below.
  double x {0.};  //!< Position of the detection in the frame above [m].
  double y {0.};
  double z {0.};
  double qx {0.};  //!< Orientation of the detection in the frame above.
  double qy {0.};
  double qz {0.};
  double qw {1.};
  bool hasPose {false};  //!< Whether the position above is known.
  std::string ecefFrame;  //!< Frame the global position below is expressed in (empty when unknown).
  double ecefX {0.};  //!< Position of the detection in the global (ECEF) frame [m].
  double ecefY {0.};
  double ecefZ {0.};
  double ecefQx {0.};  //!< Orientation of the detection in the global frame.
  double ecefQy {0.};
  double ecefQz {0.};
  double ecefQw {1.};
  bool hasEcefPosition {false};  //!< Whether the global position above is known.
  double bboxX {0.};  //!< Centre of the 2D bounding box in the source image [px].
  double bboxY {0.};
  double bboxW {0.};  //!< Size of the 2D bounding box in the source image [px].
  double bboxH {0.};
  bool hasBbox {false};  //!< Whether the bounding box above is known.
  std::string zoneFrame;  //!< Frame the keep-out zones (and the position below) are defined in.
  double zoneX {0.};  //!< Position of the detection in the zones frame [m].
  double zoneY {0.};
  double zoneZ {0.};
  bool hasZonePosition {false};  //!< Whether the position in the zones frame is known.
  std::string zoneStatus;  //!< in / out / unknown - membership in the keep-out zones.
  std::vector<int> zoneIds;  //!< Numbers of the zones the detection is in.
  std::string imagePath;  //!< Path of the stored image of the detection (empty if there is none).
  unsigned imageWidth {0};  //!< Width of the stored image [px].
  unsigned imageHeight {0};  //!< Height of the stored image [px].
  std::string imageFrame;  //!< Frame of the camera the image was taken by (empty in the old storage format).
  std::string cameraFrame;  //!< Frame of the camera the detection was reported in (empty if not recorded).
  //! Whether to show the detection at its global position instead of in the frame it was recorded in.
  bool preferGlobalFrame {false};

  //! Whether the detection was recorded inside at least one keep-out zone.
  bool inZone() const {
    return zoneStatus == "in";
  }

  //! Whether the global (ECEF) position is the one to visualize.
  bool usesGlobalFrame() const {
    return preferGlobalFrame && hasEcefPosition;
  }

  //! Frame to visualize the detection in (the global one, or the zones frame when its position is known).
  const std::string& displayFrame() const {
    if (usesGlobalFrame()) {
      return ecefFrame;
    }
    return hasZonePosition ? zoneFrame : frameId;
  }

  //! Whether the record has a position to visualize.
  bool hasDisplayPose() const {
    return usesGlobalFrame() || hasZonePosition || hasPose;
  }

  /**
   * \brief Which frame and position to visualize.
   *
   * The global (ECEF) position is preferred when the caller asked for it because it is the only one which stays
   * valid between missions. Otherwise the position in the zones frame (typically the map frame of the current
   * mission) is preferred, because it is directly comparable to the rest of the scene and does not depend on a live
   * transform from the sensor frame.
   *
   * \param[out] position the position in displayFrame().
   * \param[out] orientation the orientation to use (the stored one for the global position, identity otherwise).
   */
  void getDisplayPose(double position[3], double orientation[4]) const;
};

/**
 * \brief What to load from the detection storage.
 */
struct StorageQuery {
  std::string rootDir;  //!< Root of the storage, empty for the default location.
  int64_t startNs {0};  //!< Oldest detection to load (inclusive).
  //! Newest detection to load (inclusive). Defaults to the end of time so that a query without an explicit end
  //! loads everything up to now.
  int64_t endNs {std::numeric_limits<int64_t>::max()};
  std::vector<std::string> classes;  //!< Classes to load, empty for all of them.
  //! Cameras (image frames) to load, empty for all of them. Several cameras can share the topic the detections are
  //! recorded from, and their frames are the only thing to tell them apart.
  std::vector<std::string> cameras;
  //! Keep only detections in a keep-out zone (1), outside of all of them (-1), or all of them (0).
  int zoneFilter {0};
  size_t limit {0};  //!< Maximal number of records to return (the newest ones), 0 for all of them.
};

/**
 * \brief Reading of the persistent storage of detections written by g4s_mission.
 *
 * The storage contains one CSV file per day (\c detections_YYYY-MM-DD.csv) with appended rows, and the images of the
 * detections in \c images/YYYY-MM-DD/<camera>/<class>/<id>.jpg. The reader is tolerant to incomplete and malformed rows
 * (a day file may be read while it is still being written), and it never throws.
 */
class DetectionStore {
public:
  /**
   * \brief The default location of the detection storage.
   *
   * It is the \c g4s_mission folder in \c $ROS_HOME (which is \c ~/.ros when the variable is not set), the same
   * default the detection recorder uses.
   *
   * \return the path of the storage root.
   */
  static std::string defaultRootDir();

  /**
   * \brief Expands a leading \c ~ in the given path by the home folder of the current user.
   * \param[in] path the path to expand.
   * \return the expanded path.
   */
  static std::string expandUser(const std::string& path);

  /**
   * \brief Splits one line of a CSV file into its fields.
   *
   * Quoted fields (which may contain commas, and doubled quotes to represent a quote) are supported.
   *
   * \param[in] line the line to split.
   * \return the fields of the line, without the quoting.
   */
  static std::vector<std::string> splitLine(const std::string& line);

  /**
   * \brief The CSV files of the storage that can contain detections of the given period, oldest first.
   * \param[in] rootDir root of the storage.
   * \param[in] startNs oldest interesting detection.
   * \param[in] endNs newest interesting detection.
   * \return the paths of the files that exist.
   */
  static std::vector<std::string> csvFiles(const std::string& rootDir, int64_t startNs, int64_t endNs);

  /**
   * \brief Names of the sub-directories of the given directory which are not hidden.
   *
   * Used to walk the folders of the image storage, whose names come from the sensor data (dates, camera frames and
   * class names) and are therefore unknown in advance.
   *
   * \param[in] dir directory to look into.
   * \return the names of the folders in it, sorted (an empty list if it does not exist).
   */
  static std::vector<std::string> listFiles(const std::string& dir);

  //! File extensions of the stored detection images, in the order in which they are preferred.
  static const std::vector<std::string>& imageExtensions();

  /**
   * \brief Parses one row of a detection CSV file.
   * \param[in] row values of the row, indexed by the column name.
   * \param[in] rootDir root of the storage, used to resolve the path of the detection image.
   * \param[out] record the parsed record.
   * \return false if the row does not describe a usable detection (it has no valid stamp).
   */
  static bool parseRow(const std::map<std::string, std::string>& row, const std::string& rootDir,
      StoredRecord& record);

  /**
   * \brief Loads the stored detections matching the given query.
   * \param[in] query what to load.
   * \param[out] error description of the problem with the storage (empty if it is readable).
   * \param[out] skipped number of rows that could not be parsed (e.g. a row which is still being written).
   * \return the matching records, sorted by their stamp.
   */
  static std::vector<StoredRecord> load(const StorageQuery& query, std::string& error, size_t* skipped = nullptr);
};

}  // namespace cras_rviz_vision_msgs
