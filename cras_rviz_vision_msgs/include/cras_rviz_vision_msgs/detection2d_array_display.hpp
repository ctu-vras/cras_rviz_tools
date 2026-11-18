// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief RViz display for vision_msgs/Detection2DArray messages.
 * \author Czech Technical University in Prague
 */

#pragma once

#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <cras_cpp_common/image_transport.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <image_transport/subscriber.hpp>
#include <OgreManualObject.h>
#include <OgreSceneNode.h>
#include <rclcpp/subscription.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/properties/status_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/ros_topic_display.hpp>
#include <rviz_common/visibility_control.hpp>
#include <rviz_rendering/objects/movable_text.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include "cras_rviz_vision_msgs/detection_buffer.hpp"
#include "cras_rviz_vision_msgs/detection_store.hpp"
#include "cras_rviz_vision_msgs/image_cache.hpp"
#include "cras_rviz_vision_msgs/image_topic_property.hpp"
#include "cras_rviz_vision_msgs/layout_planner.hpp"

namespace cras_rviz_vision_msgs {

/**
 * \brief Visualization of detections in the 2D image plane (vision_msgs/Detection2DArray).
 *
 * Each detection is visualized at the position of its most probable class hypothesis. Because the hypotheses come with
 * a 3D pose estimate, the detections can be shown in the 3D scene and projected into the currently active view, which
 * works both with the TopDownOrtho camera (the typical use case) and with any other camera. The position of the
 * detection is marked by a small solid sphere, the same way the PointStamped display shows a point.
 *
 * The detections are kept also after the message that brought them is gone - a detector typically publishes the same
 * objects repeatedly and publishes an empty array when it sees nothing, so neither showing only the contents of the
 * newest message nor hiding the detections on an empty message would be useful. The detections published again with the
 * same ID are updated in place and the oldest detections are dropped when the (configurable, by default a few
 * thousands) limit is exceeded or when they expire.
 *
 * To keep the view readable even when a lot of detections are close to each other, only a limited number of detections
 * can be shown with an image preview (the part of the source camera image around the detection bounding box). The
 * preview is attached to the detection as a quad that always faces the camera. Which detections get the preview is
 * decided by LayoutPlanner: the most probable detections (and those in the centre of the view) win, and when the user
 * zooms in, more space becomes available in the view and thus more previews appear. The remaining detections are only
 * plotted as a plain marker with a text label.
 */
class Detection2DArrayDisplay
    : public rviz_common::RosTopicDisplay<vision_msgs::msg::Detection2DArray> {
  Q_OBJECT

public:
  //! How the color of the detection marker is chosen.
  enum class ColorMode{
    PROBABILITY = 0,  //!< From the detection probability (green for high, red for low).
    CLASS_NAME = 1,  //!< Hashed from the class name (the same class always has the same color).
    NONE = 2,  //!< Constant color of all detections.
    ZONE = 3,  //!< From the membership in a keep-out zone (red inside one, blue outside of all of them).
  };

  Detection2DArrayDisplay();
  ~Detection2DArrayDisplay() override;

  void onInitialize() override;
  void update(float wall_dt, float ros_dt) override;
  void reset() override;

  //! Number of visualized detections. Only meant for testing and debugging.
  size_t getVisualizationSize() const;
  //! Number of detections that are currently shown with an image preview. Only meant for testing and debugging.
  size_t getPreviewCount() const;
  //! Number of detections loaded from the persistent storage. Only meant for testing and debugging.
  size_t getHistoryCount() const;

protected:
  //! A detection, either as it was received in a message, or as it was read from the persistent storage.
  using StoredDetection = cras_rviz_vision_msgs::StoredDetection;

  void processMessage(vision_msgs::msg::Detection2DArray::ConstSharedPtr msg) override;

  //! Draws the detections into the currently active view. Called by update().
  void drawDetections(Ogre::Camera& camera, Ogre::Viewport& viewport);

  //! Callback for the incoming camera images.
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

  //! Callback for the incoming compressed camera images.
  void compressedImageCallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& msg);

  /**
   * \brief The current time, in the time base of the detections.
   *
   * It is the time reported by the ROS clock, so that the age of the detections (and the time range of the loaded
   * history) is relative to the simulation time when the clock is driven by \c /clock. When the clock is not driven
   * yet, the wall clock is used instead and \c rosClock is set to false.
   *
   * \param[out] rosClock Can be null. Set to true if the returned time comes from the ROS clock.
   */
  int64_t nowNs(bool* rosClock = nullptr) const;

  //! Returns the color of a detection according to the selected color mode.
  Ogre::ColourValue getColor(const StoredDetection& detection) const;

  //! Returns the text describing the given detection (empty if labels are disabled).
  std::string getLabel(const StoredDetection& detection) const;

  //! Creates the material of the detection markers if it does not exist yet.
  void createMarkerMaterial();

  //! Reads the historical detections from the persistent storage and replaces the currently loaded ones.
  void loadHistory();

  //! Forgets the detections loaded from the persistent storage, including their images.
  void clearHistory();

  //! The query describing what should be loaded from the persistent storage.
  StorageQuery historyQuery() const;

  /**
   * \brief The image of a detection loaded from the persistent storage.
   *
   * The images are cached because the annotations are redrawn every frame and reading them from the disk each time
   * would be much too slow.
   *
   * \param[in] path the path of the stored image, empty if the detection has none.
   * \return the image, or an invalid image if it cannot be loaded.
   */
  const CameraImage& historyImage(const std::string& path);

  //! Removes everything drawn by the previous call of drawDetections().
  void clearOverlays();

  //! A text label attached to a detection.
  struct Label{
    std::unique_ptr<rviz_rendering::MovableText> text;  //!< The drawn text.
  };

  std::unique_ptr<ImageCache> images_;  //!< Cache of the camera images.
  std::unique_ptr<TextureCache> textures_;  //!< Cache of the image previews.
  std::unique_ptr<cras::ImageTransport> imageTransport_;  //!< Camera image transport.
  image_transport::Subscriber imageSubscriber_;  //!< Camera image subscriber.
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressedSubscriber_;  //!< Compressed images.
  std::string imageTopicName_;  //!< The image topic the display is currently subscribed to.
  bool imageTopicCompressed_ {false};  //!< Whether that topic carries compressed images.
  std::string detectionsTopicName_;  //!< The topic the accumulated detections come from.

  //! Camera image topic, offering both the raw and the compressed image topics.
  ImageTopicProperty* imageTopicProperty_{nullptr};
  rviz_common::properties::FloatProperty* imageToleranceProperty_{nullptr};
  rviz_common::properties::IntProperty* cacheSizeProperty_{nullptr};
  rviz_common::properties::FloatProperty* markerSizeProperty_{nullptr};
  rviz_common::properties::FloatProperty* markerThicknessProperty_{nullptr};
  rviz_common::properties::BoolProperty* accumulateProperty_{nullptr};
  rviz_common::properties::IntProperty* maxDetectionsProperty_{nullptr};
  rviz_common::properties::FloatProperty* lifetimeProperty_{nullptr};
  rviz_common::properties::IntProperty* maxLabelsProperty_{nullptr};
  rviz_common::properties::FloatProperty* minProbabilityProperty_{nullptr};
  rviz_common::properties::EnumProperty* colorModeProperty_{nullptr};
  rviz_common::properties::BoolProperty* showLabelsProperty_{nullptr};
  rviz_common::properties::BoolProperty* showClassProperty_{nullptr};
  rviz_common::properties::BoolProperty* showProbabilityProperty_{nullptr};
  rviz_common::properties::BoolProperty* showIdProperty_{nullptr};
  rviz_common::properties::FloatProperty* fontSizeProperty_{nullptr};
  rviz_common::properties::StringProperty* fontProperty_{nullptr};
  rviz_common::properties::BoolProperty* showImagesProperty_{nullptr};
  rviz_common::properties::FloatProperty* previewPaddingProperty_{nullptr};
  rviz_common::properties::BoolProperty* avoidOverlapsProperty_{nullptr};
  rviz_common::properties::FloatProperty* spacingProperty_{nullptr};
  rviz_common::properties::IntProperty* maxPreviewWidthProperty_{nullptr};
  rviz_common::properties::IntProperty* minPreviewWidthProperty_{nullptr};
  rviz_common::properties::IntProperty* maxPreviewsProperty_{nullptr};
  rviz_common::properties::BoolProperty* showTimeProperty_{nullptr};
  rviz_common::properties::BoolProperty* showZoneProperty_{nullptr};

  rviz_common::properties::BoolProperty* historyEnabledProperty_{nullptr};
  rviz_common::properties::StringProperty* storageDirProperty_{nullptr};
  rviz_common::properties::StringProperty* globalFrameProperty_{nullptr};
  rviz_common::properties::IntProperty* historyFromProperty_{nullptr};
  rviz_common::properties::IntProperty* historyToProperty_{nullptr};
  rviz_common::properties::IntProperty* historyLimitProperty_{nullptr};
  rviz_common::properties::StringProperty* historyClassesProperty_{nullptr};
  rviz_common::properties::EnumProperty* historyZoneProperty_{nullptr};
  rviz_common::properties::BoolProperty* historyImagesProperty_{nullptr};
  rviz_common::properties::FloatProperty* historyReloadProperty_{nullptr};

  DetectionBuffer detections_{DetectionBuffer::DEFAULT_LIMIT};  //!< The received detections (guarded by the mutex).
  std::vector<StoredDetection> history_;  //!< The detections loaded from the persistent storage.
  std::map<std::string, CameraImage> historyImages_;  //!< Cache of the images loaded for the history detections.
  std::vector<std::string> historyImageOrder_;  //!< Order in which the cached history images were used.
  std::string frameId;  //!< Frame of the last message.
  std::string historyError_;  //!< What is wrong with the persistent storage (empty if it is readable).
  size_t historySkipped_{0};  //!< Number of stored rows which could not be read.
  double historyAge_{0.};  //!< Time since the history was loaded (used by the periodic reload).
  std::atomic<bool> historyDirty_{true};  //!< Whether the history should be loaded again.
  size_t historyLoaded_{0};  //!< Number of detections currently loaded from the storage.
  //! The images of the stored detections are guarded by no lock, they are used (and cached) only in the rendering
  //! thread, unlike the detections above, which also arrive in the ROS callbacks thread.
  mutable std::mutex detectionsMutex_;  //!< Guards the members above (history_ included).

  Ogre::SceneNode* overlayNode_{nullptr};  //!< Node with all the drawn annotations.
  Ogre::ManualObject* markerObject_{nullptr};  //!< The drawn detection markers.
  Ogre::ManualObject* previewObject_{nullptr};  //!< The drawn image previews.
  std::vector<std::unique_ptr<Label>> labels_;  //!< The drawn text labels.
  size_t previewCount_{0};  //!< Number of drawn image previews.
  size_t visualizedCount_{0};  //!< Number of drawn detections.
  size_t poselessCount_{0};  //!< Number of detections that have no 3D pose.
  size_t untransformableCount_{0};  //!< Number of detections that cannot be transformed to the view.
  size_t untransformableGlobalCount_{0};  //!< Number of detections that cannot be transformed to global frame.
  bool textFailed_{false};  //!< Whether creating the text labels failed.
  std::string lastStatusKey_;  //!< State the current status was computed for.

protected Q_SLOTS:
  //! (Re)subscribes to the image topic.
  void updateImageTopic();
  //! Asks for the historical detections to be loaded again (called when the storage settings change).
  void reloadHistory();
  //! Applies the number of cached images.
  void updateCacheSize();
  //! Draws the detections again (called when any of the visualization properties changes).
  void redraw();
  //! Reports the state of the inputs in the display status.
  void updateStatus();
};

}  // namespace cras_rviz_vision_msgs
