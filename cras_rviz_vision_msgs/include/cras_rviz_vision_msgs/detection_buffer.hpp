// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Buffer of detections to visualize.
 * \author Czech Technical University in Prague
 */

#pragma once

#include <algorithm>
#include <cstdint>
#include <list>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/time.hpp>

#include <cras_rviz_vision_msgs/image_cache.hpp>

namespace cras_rviz_vision_msgs {

/**
 * \brief How close a new detection has to be to an existing one for them to be considered the same detection.
 *
 * All the thresholds have to be satisfied for the new detection to replace the existing one. A non-positive
 * \ref maxDistance3d turns the whole filter off (the detections are then only told apart by their IDs).
 */
struct Deduplication {
  double maxDistance3d {1.};  //!< Maximal distance of the 3D positions [m].
  double maxDistance2d {50.};  //!< Maximal distance of the centres of the 2D bounding boxes [px].
  double maxTimeDiff {5.};  //!< Maximal difference of the detection stamps [s].

  //! Whether the filter is enabled.
  bool enabled() const {
    return maxDistance3d > 0.;
  }
};

/**
 * \brief A place of a camera image the user points at, to find the detection which was seen there.
 *
 * It is either the header of the camera image (the user clicked somewhere in it without caring where), or a point in
 * its pixel coordinates, which the detection bounding boxes can be compared to. See \ref DetectionBuffer::select.
 */
struct Selection {
  std::string cameraFrame;  //!< Frame of the camera the click happened in (empty when not known).
  rclcpp::Time stamp {0, 0};  //!< Stamp of the camera image the click happened in.
  double tolerance {2.};  //!< Maximal difference of the detection and the image stamps [s].
  bool hasPoint {false};  //!< Whether the position below is known (a clicked point, not only an image).
  double x {0.};  //!< Horizontal position of the click in the image [px].
  double y {0.};  //!< Vertical position of the click in the image [px].
  double maxDistance {40.};  //!< Maximal distance of the click from a bounding box which still belongs to it [px].
};

/**
 * \brief One detection kept for visualization.
 */
struct StoredDetection {
  rclcpp::Time stamp {0, 0};  //!< Stamp of the message the detection arrived in (also of its camera image).
  int64_t receivedNs {0};  //!< When the detection was last confirmed by a message (nanoseconds since epoch).
  std::string id;  //!< ID of the detection in the messages (may be empty).
  std::string frameId;  //!< Frame of the pose below.
  /**
   * \brief Frame the detection arrived in, which never changes.
   *
   * The pose of a detection is moved to the global frame as soon as the transform is available (and the frame of its
   * pose changes with it), but what the detector reports again is a detection in the frame it came in. The frame of the
   * pose therefore cannot tell which detection a message reports again - this frame is there for that.
   */
  std::string sourceFrameId;
  std::string cameraFrame;  //!< Frame of the camera the detection was reported in (empty when not known).
  bool inGlobalFrame {false};  //!< Whether the pose is in the global frame, whose transform never expires.
  int64_t resolveTryNs {0};  //!< When the pose was last tried to be transformed to the global frame.
  int64_t zoneTryNs {0};  //!< When the keep-out zones were last tried to be evaluated for this detection.
  geometry_msgs::msg::Pose pose;  //!< Position of the detection (the pose of its most probable class).
  bool hasPose {false};  //!< Whether the pose above is known.
  std::string className;  //!< Name of the most probable class.
  double probability {1.};  //!< Probability of the most probable class.
  ImageRegion imageRegion;  //!< Position of the detection in the source image (zero size when unknown).
  rclcpp::Time previewStamp {0, 0};  //!< Stamp of the camera image the preview of the detection is taken from.
  ImageRegion previewRegion;  //!< Which part of that image to show.
  int64_t previewNs {0};  //!< When the preview was taken (so that it does not refresh too often).
  bool previewTaken {false};  //!< Whether the preview above has already been taken from a camera image.
  bool history {false};  //!< Whether the detection comes from the persistent storage.
  std::string imagePath;  //!< Image of the detection from the persistent storage (empty otherwise).
  std::vector<int> zoneIds;  //!< Numbers of the keep-out zones the detection is in.
  bool hasZone {false};  //!< Whether the keep-out zone information above is known.
  bool inZone {false};  //!< Whether the detection is inside a keep-out zone.
  /**
   * \brief Whether the detection lies in one of the keep-out zones known \b now.
   *
   * Unlike the membership above, which is the one the detection arrived with (or the one it was recorded with), this
   * one is evaluated again whenever the zones or the position of the detection change. It is unknown (\c nullopt)
   * for a detection whose position in the frame of the zones is not known.
   */
  std::optional<bool> inZoneNow;
  std::vector<int> nowZoneIds;  //!< Numbers of the zones the detection lies in now.
  uint64_t serial {0};  //!< Unique number of the detection, kept as long as the detection is remembered.
};

/**
 * \brief Whether a detection belongs to a keep-out zone.
 * \return What the zones known now say about the detection, falling back to the membership it arrived (or was
 *   recorded) with when they cannot be told for it.
 */
inline bool insideZone(const StoredDetection& detection) {
  return detection.inZoneNow ? *detection.inZoneNow : (detection.hasZone && detection.inZone);
}

/**
 * \brief Detections to visualize, with a bounded number of them.
 *
 * The buffer keeps the detections received on the topic (and optionally also the detections read from the persistent
 * storage) without any time limit, so that a detection does not disappear when the detector stops reporting it.
 * Because that would fill the memory with thousands of almost identical detections, a new detection which is close to
 * an existing one (see \ref Deduplication) replaces it instead of being added. The oldest detections are dropped when
 * their number exceeds the limit.
 *
 * Each detection has a serial number which stays the same as long as the detection is remembered (and which the
 * detection replacing it takes over), so that the rendering resources of a detection can be reused even when the
 * buffer has been reorganised.
 */
class DetectionBuffer {
public:
  static constexpr size_t DEFAULT_LIMIT = 1000;  //!< Number of detections kept by default.

  DetectionBuffer();
  explicit DetectionBuffer(size_t limit);

  DetectionBuffer(const DetectionBuffer&) = delete;
  DetectionBuffer& operator=(const DetectionBuffer&) = delete;

  //! \brief Maximal number of detections kept in the buffer.
  size_t limit() const;
  //! \brief Sets the maximal number of detections and drops the oldest ones above it.
  void setLimit(size_t limit);

  //! \brief The filter deciding when a detection replaces an existing one.
  const Deduplication& deduplication() const;
  //! \brief Sets the filter and reindexes the stored detections by it.
  void setDeduplication(const Deduplication& deduplication);

  /**
   * \brief Adds new detections to the buffer.
   * \param[in] detections The detections to add; their serial numbers are assigned by the buffer.
   * \param[in] accumulate Whether to keep the detections received before. If false, only the detections from this
   *   call and the detections from the persistent storage are kept. Empty input never drops anything.
   */
  void add(std::vector<StoredDetection> detections, bool accumulate);

  /**
   * \brief Drops detections which have not been confirmed by a message for longer than their lifetime.
   *
   * A detection inside a keep-out zone is a more urgent thing than one which is out of all of them, so the two can be
   * forgotten after a different time. A detection which does not know its zone (because no zones are known) counts as
   * being out of them. The detections loaded from the persistent storage are a record of the past and their age is not
   * counted; they are dropped only when the history is loaded again, or when their number exceeds the limit.
   *
   * \param[in] nowNs Current time in nanoseconds since epoch.
   * \param[in] lifetime Maximal age of a detection which is out of any keep-out zone [s]; a non-positive value keeps
   *   all of them, whatever their age is.
   * \param[in] inZoneLifetime Maximal age of a detection which is inside a keep-out zone (see \ref insideZone) [s]; a
   *   non-positive value keeps them until they are dropped by the limit.
   * \return Number of detections dropped.
   */
  size_t prune(int64_t nowNs, double lifetime, double inZoneLifetime);

  //! \brief Drops all detections, including the ones from the persistent storage.
  void clear();
  //! \brief Drops the detections received on the topic and keeps the ones from the persistent storage.
  size_t clearLive();
  //! \brief Drops the detections loaded from the persistent storage and keeps the ones from the topic.
  size_t clearHistory();

  //! \brief Puts all stored detections into the indexes again; see \ref detections().
  void reindex();

  /**
   * \brief The stored detections. They may be modified (e.g. their pose may be transformed to another frame).
   *
   * Changing the identification of a detection (its ID, the frame of its pose or the camera it came from) or its pose
   * this way has to be followed by \ref reindex, otherwise the buffer would not find the detection again and the same
   * object would be remembered (and drawn) twice.
   */
  std::list<StoredDetection>& detections();
  const std::list<StoredDetection>& detections() const;
  size_t size() const;
  //! \brief Number of detections dropped because of the limit.
  size_t dropped() const;
  //! \brief Number of detections which were replaced by a newer detection close to them.
  size_t replaced() const;
  //! \brief Number of detections which were dropped by the age limit.
  size_t expired() const;
  /**
   * \brief Time of the newest detection in the buffer (nanoseconds since epoch), 0 when the buffer is empty.
   *
   * The detections are not stored in the order they were stamped in, so the newest one has to be looked for.
   */
  int64_t newestStamp() const;
  /**
   * \brief Counter incremented whenever the content of the buffer changes.
   *
   * Lets the user notice a change without comparing the detections.
   */
  uint64_t revision() const;

  /**
   * \brief The detections which are (probably) the object the user pointed at in a camera image.
   *
   * A detection is a candidate when it was reported by the requested camera (or does not know its camera) and its stamp
   * is close to the stamp of the clicked image. When the point is known, it also has to be inside a bounding box, or
   * close enough to one (see \ref Selection::maxDistance); the detections which contain it are preferred, and among
   * them the smallest one, which is the object the user could have seen.
   *
   * \param[in] selection Where and when the user pointed.
   * \return Serial numbers of the matching detections, the most likely one first.
   */
  std::vector<uint64_t> select(const Selection& selection) const;

  /**
   * \brief Makes a detection show another camera image than the one it was first seen in.
   *
   * The preview of a detection sticks to the image it was first seen in, which is what keeps the visualization calm
   * while an object is reported over and over again. When the user points at an object in a newer image, it is that
   * image they look at and want the detection to show, so the preview is taken again from the given one.
   *
   * \param[in] serial Serial number of the detection.
   * \param[in] selection Where the user clicked; its stamp is the camera image the detection should show, and its place,
   *   when it has one, is the part of that image the preview has to show.
   * \return Whether the detection exists and its preview was taken again.
   */
  bool retakePreview(uint64_t serial, const Selection& selection);

protected:
  /**
   * \brief Same as \ref select, with the clicked place optionally left out of the matching.
   *
   * \param[in] selection Where and when the user pointed.
   * \param[in] usePlace Whether a detection which does not contain the clicked place should be excluded.
   * \param[in] requireCamera Whether only detections reported by the clicked camera are candidates.
   * \return Serial numbers of the matching detections, the most likely one first.
   */
  std::vector<uint64_t> select(const Selection& selection, bool usePlace, bool requireCamera) const;

  using Detections = std::list<StoredDetection>;  //!< Storage of the detections, oldest first.
  using Iterator = Detections::iterator;  //!< Points to one stored detection.
  using Index = std::unordered_map<std::string, Iterator>;  //!< Identification of a detection to its place.
  using Grid = std::unordered_map<uint64_t, std::vector<Iterator>>;  //!< One cell of the position index.

  //! Identification of a detection which has a known ID (the same ID in another frame is another detection).
  static std::string key(const StoredDetection& detection);
  //! Key of the cell of the position index the given position falls into.
  uint64_t cellKey(double x, double y, double z) const;

  //! Size of one cell of the position index (the maximal distance a detection may be from the one it replaces).
  double cellSize() const {
    return std::max(1e-6, deduplication_.maxDistance3d);
  }

  //! Whether the new detection is close enough to the stored one for them to be the same detection.
  static bool isSimilar(const StoredDetection& stored, const StoredDetection& fresh, const Deduplication& rules);

  /**
   * \brief Decides which camera image the detection should show as its preview.
   *
   * The detection comes with the information from the newest message; the preview fields of it are kept as they were
   * (the fields of the detection it replaced are copied into it before the call), and are only taken again when the
   * detection moved away from the image it shows.
   */
  static void choosePreview(StoredDetection& detection);

  //! Takes the preview of the detection which the new detection replaces, and refreshes it if it does not fit anymore.
  static void adoptPreview(StoredDetection& fresh, const StoredDetection& previous);
  //! The stored detection the new detection should replace, or \c end() if it should be added.
  Iterator findSimilar(const StoredDetection& fresh);
  //! Puts the detection into the position index (does nothing when the filter is off or the pose is unknown).
  void index(Iterator detection);
  //! Takes the detection out of the position index (when its position changed).
  void unindex(Iterator detection);
  //! Takes the detection out of both indexes and removes it from the buffer.
  void erase(Iterator detection);
  //! Drops either the detections from the topic, or the ones from the persistent storage.
  size_t eraseGroup(bool history);

  size_t limit_;  //!< Maximal number of detections.
  Deduplication deduplication_;  //!< When a detection replaces an existing one.
  Detections detections_;  //!< The stored detections, oldest first.
  Index byId_;  //!< Index of the detections which have a known ID.
  //! Index of the detections by their position, one for the received ones and one for the stored ones, so that a
  //! detection from the storage is never replaced by a live one (the user wants to see both of them).
  std::array<Grid, 2> grids_;

  //! The position index of the given group of detections.
  Grid& grid(bool history) {
    return grids_[history ? 1U : 0U];
  }
  const Grid& grid(bool history) const {
    return grids_[history ? 1U : 0U];
  }
  size_t dropped_ {0};  //!< Number of detections dropped because of the limit.
  size_t replaced_ {0};  //!< Number of detections replaced by a newer one.
  size_t expired_ {0};  //!< Number of detections dropped because they were too old.
  uint64_t serialCounter_ {0};  //!< Assigns the serial numbers.
  uint64_t revision_ {0};  //!< Increments with every change of the content.
};

}  // namespace cras_rviz_vision_msgs
