// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Implementation of the visualization of vision_msgs/Detection2DArray messages.
 */

#include "cras_rviz_vision_msgs/detection2d_array_display.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <OgreCamera.h>
#include <OgreException.h>
#include <OgreHardwareBufferManager.h>
#include <OgreImage.h>
#include <OgreMaterial.h>
#include <OgreMaterialManager.h>
#include <OgreQuaternion.h>
#include <OgreSceneManager.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>
#include <OgreViewport.h>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/view_controller.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_rendering/geometry.hpp>
#include <rviz_rendering/render_window.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace cras_rviz_vision_msgs {

namespace {

//! Material of the detection markers.
constexpr const char* const MARKER_MATERIAL = "cras_rviz_vision_msgs/marker";
//! Maximal size of the cached camera images (in pixels).
constexpr size_t MAX_IMAGE_SIZE = 1280;
//! Maximal number of the images of the stored detections kept in memory.
constexpr size_t MAX_HISTORY_IMAGES = 32;
//! Maximal number of the image previews kept as textures in the graphics memory.
constexpr size_t MAX_PREVIEW_TEXTURES = 128;
//! Number of segments of the drawn sphere (it is drawn as a disc in the plane of the screen).
constexpr unsigned SPHERE_SEGMENTS = 12;
//! More detections than this are drawn without the light spot, to keep the number of drawn vertices reasonable.
constexpr size_t SHADED_MARKER_LIMIT = 300;
//! A detection is never drawn smaller than this, so that it does not disappear when it is far away from the camera.
constexpr double MIN_MARKER_SIZE = 3.;  // [px]
//! How long to wait for the simulated clock before determining the time range of the loaded history by the wall clock.
constexpr double CLOCK_WAIT_FOR_ROS_TIME = 3.;  // [s]

//! Converts the quaternion of the given ROS orientation to an Ogre quaternion.
inline Ogre::Quaternion toOgre(const geometry_msgs::msg::Quaternion& orientation) {
  return Ogre::Quaternion(orientation.w, orientation.x, orientation.y, orientation.z);
}

//! Converts the given colour from the HSV space to the RGB space.
inline Ogre::ColourValue hsvToColour(float hue, float saturation, float value) {
  const auto component = [hue, saturation, value](int n) {
                           const auto k = std::fmod(static_cast<float>(n) + 4.f * hue, 6.f);
                           return value - value * saturation * std::min(std::max(k, 0.f), std::min(4.f - k, 1.f));
                         };
  return Ogre::ColourValue(component(5), component(3), component(1), 1.f);
}

//! Converts the position of the given ROS point to an Ogre vector.
inline Ogre::Vector3 toOgre(const geometry_msgs::msg::Point& position) {
  return Ogre::Vector3(position.x, position.y, position.z);
}

//! Formats a detection stamp for the label of a historical detection.
std::string formatStamp(int64_t stampNs) {
  const auto seconds = static_cast<time_t>(stampNs / 1000000000LL);
  std::tm moment{};
  localtime_r(&seconds, &moment);
  char buffer[32];
  if (std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &moment) == 0) {
    return std::to_string(seconds);
  }
  return buffer;
}

}  // namespace

Detection2DArrayDisplay::Detection2DArrayDisplay() {
  imageTopicProperty_ = new ImageTopicProperty("Image topic",
      "Camera images the detections come from. The part of the image around the detection bounding box is shown as "
      "the detection preview. Both sensor_msgs/msg/Image and sensor_msgs/msg/CompressedImage topics can be used.",
      this, SLOT(updateImageTopic()));

  imageToleranceProperty_ = new rviz_common::properties::FloatProperty("Image time tolerance", 0.15,
      "Maximal time difference between the detection message and the camera image to show the image preview [s].",
      this, SLOT(updateStatus()));
  imageToleranceProperty_->setMin(0.);
  imageToleranceProperty_->setMax(10.);
  cacheSizeProperty_ = new rviz_common::properties::IntProperty("Images to cache", 4,
      "How many last camera images to keep in memory for the image previews.", this, SLOT(updateCacheSize()));
  cacheSizeProperty_->setMin(1);
  cacheSizeProperty_->setMax(32);

  auto* markerCategory = new rviz_common::properties::Property("Markers", "",
      "How the detections themselves are drawn.", this);
  markerSizeProperty_ = new rviz_common::properties::FloatProperty("Size", .1,
      "Diameter of the drawn sphere [m]. It is drawn in the same way the PointStamped display draws a point, i.e. with "
      "a size given in the scene units, so it grows when the user zooms in.", markerCategory);
  markerSizeProperty_->setMin(0.001);
  markerSizeProperty_->setMax(100.);
  markerThicknessProperty_ = new rviz_common::properties::FloatProperty("Line width", 1.,
      "Width of the outline of the drawn sphere [px]. Set 0 to draw only the filled sphere.", markerCategory);
  markerThicknessProperty_->setMin(0.);
  markerThicknessProperty_->setMax(20.);
  accumulateProperty_ = new rviz_common::properties::BoolProperty("Keep older detections", true,
      "Whether the detections from the older messages should stay visible. When enabled, an empty message does not "
      "hide the detections and the display can show more detections than a single message contains. A detection which "
      "is published again with the same ID is updated instead of being drawn twice.", markerCategory);
  maxDetectionsProperty_ = new rviz_common::properties::IntProperty("Max. detections", 2000,
      "Maximal number of the kept detections. When more of them is received, the oldest ones are dropped.",
      markerCategory);
  maxDetectionsProperty_->setMin(1);
  maxDetectionsProperty_->setMax(1000000);
  lifetimeProperty_ = new rviz_common::properties::FloatProperty("Detections lifetime", 0.,
      "How long the detections are kept, in seconds. Detections older than this are dropped. 0 keeps them until the "
      "limit above is exceeded.", markerCategory);
  lifetimeProperty_->setMin(0.);
  lifetimeProperty_->setMax(86400.);
  minProbabilityProperty_ = new rviz_common::properties::FloatProperty("Min. probability", 0.,
      "Detections with a lower probability than this are not visualized.", markerCategory);
  minProbabilityProperty_->setMin(0.);
  minProbabilityProperty_->setMax(1.);
  colorModeProperty_ = new rviz_common::properties::EnumProperty("Color by", "Probability",
      "How the color of the detection marker is chosen.", markerCategory);
  colorModeProperty_->addOption("Probability", static_cast<int>(ColorMode::PROBABILITY));
  colorModeProperty_->addOption("Class name", static_cast<int>(ColorMode::CLASS_NAME));
  colorModeProperty_->addOption("Constant", static_cast<int>(ColorMode::NONE));
  colorModeProperty_->addOption("Keep-out zone", static_cast<int>(ColorMode::ZONE));

  auto* labelCategory = new rviz_common::properties::Property("Labels", "",
      "Text describing each detection.", this);
  showLabelsProperty_ = new rviz_common::properties::BoolProperty("Show labels", true,
      "Whether the text labels next to the detections should be shown.", labelCategory);
  showClassProperty_ = new rviz_common::properties::BoolProperty("Show class", true,
      "Whether the name of the most probable class should be part of the label.", labelCategory);
  showProbabilityProperty_ = new rviz_common::properties::BoolProperty("Show probability", true,
      "Whether the probability of the most probable class should be part of the label.", labelCategory);
  showIdProperty_ = new rviz_common::properties::BoolProperty("Show ID", false,
      "Whether the ID of the detection should be part of the label.", labelCategory);
  showTimeProperty_ = new rviz_common::properties::BoolProperty("Show time", true,
      "Whether the time of the detections loaded from the persistent storage should be part of their label.",
      labelCategory);
  showZoneProperty_ = new rviz_common::properties::BoolProperty("Show zone", true,
      "Whether the keep-out zone the detection is in (or the information that it is in none) should be part of the "
      "label.", labelCategory);
  maxLabelsProperty_ = new rviz_common::properties::IntProperty("Max. labels", 200,
      "Maximal number of labels which are shown at once. With hundreds of detections, drawing all the labels would be "
      "slow and unreadable, so only the labels of the most probable detections are shown.", labelCategory);
  maxLabelsProperty_->setMin(0);
  maxLabelsProperty_->setMax(100000);
  fontSizeProperty_ = new rviz_common::properties::FloatProperty("Font size", 14,
      "Height of the label text [px].", labelCategory);
  fontSizeProperty_->setMin(4);
  fontSizeProperty_->setMax(100);
  fontProperty_ = new rviz_common::properties::StringProperty("Font", "Liberation Sans",
      "Name of the font used for the labels.", labelCategory);

  auto* imageCategory = new rviz_common::properties::Property("Image previews", "",
      "Previews of the source camera image attached to the detections.", this);
  showImagesProperty_ = new rviz_common::properties::BoolProperty("Show image previews", true,
      "Whether the detections can show a preview of the source camera image.", imageCategory);
  previewPaddingProperty_ = new rviz_common::properties::FloatProperty("Preview padding", 1.2,
      "The preview shows a part of the image this much bigger than the detection bounding box.", imageCategory);
  previewPaddingProperty_->setMin(1.);
  previewPaddingProperty_->setMax(10.);
  maxPreviewWidthProperty_ = new rviz_common::properties::IntProperty("Preview width", 150,
      "Width of the image preview if there is enough free space around the detection [px]. The preview is made "
      "smaller (or hidden) when there is not enough space.", imageCategory);
  maxPreviewWidthProperty_->setMin(16);
  maxPreviewWidthProperty_->setMax(1024);
  minPreviewWidthProperty_ = new rviz_common::properties::IntProperty("Min. preview width", 40,
      "If there is not enough space to show a preview of at least this width, only the detection marker is drawn.",
      imageCategory);
  minPreviewWidthProperty_->setMin(8);
  minPreviewWidthProperty_->setMax(512);
  maxPreviewsProperty_ = new rviz_common::properties::IntProperty("Max. previews", 100,
      "Maximal number of detections shown with an image preview at once. Each preview needs a texture in the graphics "
      "memory, so showing all of them in a scene with thousands of detections would be too expensive. 0 is no limit.",
      imageCategory);
  maxPreviewsProperty_->setMin(0);
  maxPreviewsProperty_->setMax(10000);
  avoidOverlapsProperty_ = new rviz_common::properties::BoolProperty("Avoid overlaps", true,
      "Whether the annotations are placed so that they do not overlap each other. When disabled, every detection "
      "shows its preview centred at the detection, which can become unreadable in crowded scenes.", imageCategory);
  spacingProperty_ = new rviz_common::properties::FloatProperty("Spacing", 6.,
      "Minimal spacing between two annotations [px].", imageCategory);
  spacingProperty_->setMin(0.);
  spacingProperty_->setMax(50.);

  auto* historyCategory = new rviz_common::properties::Property("Persistent history", "",
      "Detections loaded from the persistent detection storage written by the detection recorder of the g4s_mission "
      "package.", this);
  historyEnabledProperty_ = new rviz_common::properties::BoolProperty("Show history", false,
      "Whether the stored detections should be shown together with the ones received on the topic.",
      historyCategory);
  storageDirProperty_ = new rviz_common::properties::StringProperty("Storage folder",
      QString::fromStdString(DetectionStore::defaultRootDir()),
      "Folder with the stored detections (the detections_*.csv files and the images/ subfolder). A leading ~ is "
      "replaced by the home folder.", historyCategory);
  globalFrameProperty_ = new rviz_common::properties::StringProperty("Global frame", "FP_ECEF",
      "Frame the stored detections keep their global (Earth-centred) position in, typically FP_ECEF. The stored "
      "detections are shown there, because it is the only frame which stays the same between missions. Leave empty "
      "to show them in the local frame of the mission they were recorded in.", historyCategory);
  historyFromProperty_ = new rviz_common::properties::IntProperty("Load from (s ago)", 3600,
      "Age of the oldest detection to load, in seconds. The detections are loaded up to now.", historyCategory);
  historyFromProperty_->setMin(0);
  historyFromProperty_->setMax(365 * 24 * 3600);
  historyToProperty_ = new rviz_common::properties::IntProperty("Load to (s ago)", 0,
      "Age of the newest detection to load, in seconds. Set a positive value to stop the loaded history earlier than "
      "now.", historyCategory);
  historyToProperty_->setMin(0);
  historyToProperty_->setMax(365 * 24 * 3600);
  historyLimitProperty_ = new rviz_common::properties::IntProperty("Max. stored detections", 200,
      "Maximal number of stored detections to show. When the period contains more of them, the newest ones are kept.",
      historyCategory);
  historyLimitProperty_->setMin(1);
  historyLimitProperty_->setMax(100000);
  historyClassesProperty_ = new rviz_common::properties::StringProperty("Classes", "",
      "Comma-separated list of the classes to load. Leave empty to load all of them.", historyCategory);
  historyZoneProperty_ = new rviz_common::properties::EnumProperty("Keep-out zones", "All detections",
      "Whether to show only the detections which were recorded inside a keep-out zone, or outside of all of them.",
      historyCategory);
  historyZoneProperty_->addOption("All detections", 0);
  historyZoneProperty_->addOption("Only in a zone", 1);
  historyZoneProperty_->addOption("Only out of any zone", -1);
  historyImagesProperty_ = new rviz_common::properties::BoolProperty("Use stored images", true,
      "Whether the previews of the stored detections can be taken from the images saved with them. When disabled, "
      "the stored detections are shown only with their marker and label.", historyCategory);
  historyReloadProperty_ = new rviz_common::properties::FloatProperty("Reload period", 0.,
      "How often to read the storage again, in seconds. 0 means the history is loaded only when the settings "
      "change, which is recommended for long periods.", historyCategory);
  historyReloadProperty_->setMin(0.);
  historyReloadProperty_->setMax(3600.);

  // A property sends its signals to its parent, which is the category, not the display. The display therefore has to
  // connect to the properties of the categories explicitly.
  for (auto* category : {markerCategory, labelCategory, imageCategory, historyCategory}) {
    for (int i = 0; i < category->numChildren(); ++i) {
      auto* property = category->childAt(i);
      connect(property, SIGNAL(changed()), this, SLOT(redraw()));
    }
  }
  // Anything in the query changes what is in the storage, so the history has to be loaded again.
  const std::vector<rviz_common::properties::Property*> historyProperties {
    historyEnabledProperty_, storageDirProperty_, globalFrameProperty_, historyFromProperty_, historyToProperty_,
    historyLimitProperty_, historyClassesProperty_, historyZoneProperty_};
  for (auto* property : historyProperties) {
    connect(property, SIGNAL(changed()), this, SLOT(reloadHistory()));
  }
}


Detection2DArrayDisplay::~Detection2DArrayDisplay() {
  if (imageSubscriber_) {
    imageSubscriber_.shutdown();
  }
  clearOverlays();
  if (markerObject_ && scene_manager_) {
    scene_manager_->destroyManualObject(markerObject_);
  }
  if (previewObject_ && scene_manager_) {
    scene_manager_->destroyManualObject(previewObject_);
  }
  textures_.reset();
  images_.reset();
}

void Detection2DArrayDisplay::onInitialize() {
  RTDClass::onInitialize();

  images_ = std::make_unique<ImageCache>(static_cast<size_t>(cacheSizeProperty_->getInt()));
  textures_ = std::make_unique<TextureCache>(MAX_PREVIEW_TEXTURES);

  overlayNode_ = scene_node_->createChildSceneNode();
  markerObject_ = scene_manager_->createManualObject(getName().toStdString() + "/markers");
  previewObject_ = scene_manager_->createManualObject(getName().toStdString() + "/previews");
  for (auto* object : {markerObject_, previewObject_}) {
    object->setQueryFlags(0);  // the annotations are not pickable
    object->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);
    overlayNode_->attachObject(object);
  }
  createMarkerMaterial();

  auto node = rviz_ros_node_.lock();
  if (node) {
    imageTopicProperty_->initialize(rviz_ros_node_);
    imageTransport_ = std::make_unique<cras::ImageTransport>(
        cras::ImageTransport::RequiredInterfaces(*node->get_raw_node()));
    updateImageTopic();
  }
  updateStatus();
}

void Detection2DArrayDisplay::updateImageTopic() {
  const auto topic = imageTopicProperty_->getTopicStd();
  const auto compressed = ImageTopicProperty::compressed(topic, imageTopicProperty_->selectedMessageType());
  const bool subscribed = compressed ? static_cast<bool>(compressedSubscriber_) : static_cast<bool>(imageSubscriber_);
  if (topic == imageTopicName_ && compressed == imageTopicCompressed_ && (topic.empty() || subscribed)) {
    // RViz re-emits the topic property changed signal whenever it refreshes the list of the available topics, so do
    // not disturb a running subscription (and the images it brought) unless the topic really changed.
    updateStatus();
    return;
  }
  imageTopicName_ = topic;
  imageTopicCompressed_ = compressed;
  if (imageSubscriber_) {
    imageSubscriber_.shutdown();
    imageSubscriber_ = image_transport::Subscriber();
  }
  compressedSubscriber_.reset();
  if (imageTopicProperty_ && images_) {
    images_->clear();
  }
  if (topic.empty()) {
    updateStatus();
    return;
  }
  try {
    if (compressed) {
      // The topic carries compressed images, which image_transport would only deliver when the topic they were
      // published under is known, so subscribe to them directly and let OpenCV decode them.
      const auto node = rviz_ros_node_.lock();
      if (node) {
        compressedSubscriber_ = node->get_raw_node()->create_subscription<sensor_msgs::msg::CompressedImage>(topic,
            rclcpp::QoS(rclcpp::KeepLast(2)).best_effort(),
            [this](const sensor_msgs::msg::CompressedImage::ConstSharedPtr& msg) {
              compressedImageCallback(msg);
            });
      }
    } else if (imageTransport_) {
      imageSubscriber_ = imageTransport_->subscribe(topic, rclcpp::QoS(rclcpp::KeepLast(2)),
          [this](const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
            imageCallback(msg);
          });
    }
  } catch (const std::exception& e) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("cras_rviz_vision_msgs.Detection2DArrayDisplay"),
      "Failed to subscribe to image topic \"" << topic << "\": " << e.what());
  }
  updateStatus();
}

void Detection2DArrayDisplay::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
  images_->add(msg, MAX_IMAGE_SIZE);
  queueRender();
}

void Detection2DArrayDisplay::compressedImageCallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& msg) {
  images_->add(msg, MAX_IMAGE_SIZE);
  queueRender();
}

int64_t Detection2DArrayDisplay::nowNs(bool* rosClock) const {
  // The age of the detections and the time range of the loaded history are relative to the time the detections are
  // stamped with, which is the simulation time when the clock is driven by the /clock topic.
  const auto node = rviz_ros_node_.lock();
  if (node) {
    const auto now = node->get_raw_node()->now().nanoseconds();
    if (now > 0) {
      if (rosClock) {
        *rosClock = true;
      }
      return now;
    }
  }
  if (rosClock) {
    *rosClock = false;
  }
  // The clock is driven by /clock, which has not been received yet. The wall clock is at least not the epoch.
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

void Detection2DArrayDisplay::updateCacheSize() {
  if (images_) {
    images_->setMaxImages(static_cast<size_t>(cacheSizeProperty_->getInt()));
  }
  queueRender();
}

void Detection2DArrayDisplay::redraw() {
  queueRender();
}

void Detection2DArrayDisplay::processMessage(vision_msgs::msg::Detection2DArray::ConstSharedPtr msg) {
  std::vector<StoredDetection> detections;
  detections.reserve(msg->detections.size());
  const auto global_frame = globalFrameProperty_->getStdString();
  auto frameManager = context_->getFrameManager();

  for (const auto& detection : msg->detections) {
    StoredDetection stored;
    stored.stamp = rclcpp::Time(msg->header.stamp);
    stored.id = detection.id;
    stored.frameId = msg->header.frame_id;

    // The most probable hypothesis decides which class the detection belongs to. Its pose is the 3D position of the
    // detection, which is what the visualization is anchored to.
    const vision_msgs::msg::ObjectHypothesisWithPose* best = nullptr;
    for (const auto& result : detection.results) {
      if (!best || result.hypothesis.score > best->hypothesis.score) {
        best = &result;
      }
    }
    if (best) {
      stored.className = best->hypothesis.class_id;
      stored.probability = best->hypothesis.score;
      stored.pose = best->pose.pose;
      stored.hasPose = std::isfinite(stored.pose.position.x) && std::isfinite(stored.pose.position.y) &&
                       std::isfinite(stored.pose.position.z);
      if (stored.hasPose) {
        geometry_msgs::msg::PoseStamped p;
        p.pose = stored.pose;
        p.header = msg->header;
        std::string errors;
        if (!frameManager->getTransformer()->frameHasProblems(global_frame, errors)) {
          try {
            const auto& p_global = frameManager->getTransformer()->transform(p, global_frame);
            stored.pose = p_global.pose;
            stored.frameId = p_global.header.frame_id;
          } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(
              rclcpp::get_logger("cras_rviz_vision_msgs.Detection2DArrayDisplay"),
              "Could not transform detection from frame '%s' to frame '%s': %s",
              p.header.frame_id.c_str(), global_frame.c_str(), ex.what());
            untransformableGlobalCount_++;
          }
        } else {
          RCLCPP_WARN(
            rclcpp::get_logger("cras_rviz_vision_msgs.Detection2DArrayDisplay"),
            "Could not transform detection from frame '%s' to frame '%s': %s",
            p.header.frame_id.c_str(), global_frame.c_str(), errors.c_str());
          untransformableGlobalCount_++;
        }
      }
    }

    // Position of the detection in the source image, used for the image preview.
    stored.imageRegion = {detection.bbox.center.position.x - .5 * detection.bbox.size_x,
      detection.bbox.center.position.y - .5 * detection.bbox.size_y, detection.bbox.size_x,
      detection.bbox.size_y};

    detections.push_back(std::move(stored));
  }

  {
    std::lock_guard<std::mutex> lock(detectionsMutex_);
    // The detections of the previous messages are kept, so that an empty message does not hide them and so that more
    // detections can be shown than a single message contains. A detection published again with the same ID is updated.
    if (const auto topic = topic_property_->getTopicStd(); topic != detectionsTopicName_) {
      // RViz resets the display whenever anything in its properties changes; the accumulated detections may only be
      // dropped when we start receiving another topic.
      detectionsTopicName_ = topic;
      detections_.clear();
    }
    detections_.setLimit(static_cast<size_t>(std::max(1, maxDetectionsProperty_->getInt())));
    detections_.add(std::move(detections), accumulateProperty_->getBool());
    frameId = msg->header.frame_id;
  }
  queueRender();
}

Ogre::ColourValue Detection2DArrayDisplay::getColor(const StoredDetection& detection) const{
  switch (static_cast<ColorMode>(colorModeProperty_->getOptionInt())) {
    case ColorMode::CLASS_NAME:
    {
        // Spread the classes around the colour wheel so that neighbouring classes are distinguishable.
      const auto hash = std::hash<std::string>{}(detection.className);
      const auto hue = static_cast<float>(hash % 360U) / 360.;
      return hsvToColour(hue, .8F, .95F);
    }
    case ColorMode::ZONE:
      // The same colours the alarms of the keep-out zone monitor stand for: red for a detection in a zone.
      if (!detection.hasZone) {
        return Ogre::ColourValue{.6F, .6F, .6F, 1.F};
      }
      return detection.inZone ? Ogre::ColourValue{1.F, .15F, .1F, 1.F} : Ogre::ColourValue{.2F, .8F, .3F, 1.F};
    case ColorMode::NONE: return Ogre::ColourValue{.3F, .7F, 1.F, 1.F};
    case ColorMode::PROBABILITY:
    default:
    {
      const auto score = static_cast<float>(std::max(0., std::min(1., detection.probability)));
      return Ogre::ColourValue(1.F - score, score, .1F, 1.F);
    }
  }
}

std::string Detection2DArrayDisplay::getLabel(const StoredDetection& detection) const{
  if (!showLabelsProperty_->getBool()) {
    return {};
  }
  std::string label;
  if (showClassProperty_->getBool() && !detection.className.empty()) {
    label = detection.className;
  }
  if (showProbabilityProperty_->getBool()) {
    if (!label.empty()) {
      label += ' ';
    }
    label += "(" + std::to_string(static_cast<int>(std::round(100. * detection.probability))) + "%)";
  }
  if (showIdProperty_->getBool() && !detection.id.empty()) {
    if (!label.empty()) {
      label += ' ';
    }
    label += '#' + detection.id;
  }
  // The historical detections are not marked by the passage of time in the scene, so their label says when they
  // happened and which keep-out zone they belong to.
  if (showTimeProperty_->getBool() && detection.history) {
    label += (label.empty() ? "" : " ") + formatStamp(detection.stamp.nanoseconds());
  }
  if (showZoneProperty_->getBool() && detection.hasZone) {
    std::string zone = "out";
    if (detection.inZone) {
      zone = "zone";
      for (size_t i = 0; i < detection.zoneIds.size(); ++i) {
        zone += i == 0 ? " " : ",";
        zone += std::to_string(detection.zoneIds[i]);
      }
    }
    label += (label.empty() ? "" : " ") + '[' + zone + ']';
  }
  return label;
}

void Detection2DArrayDisplay::createMarkerMaterial() {
  auto& materialManager = Ogre::MaterialManager::getSingleton();
  if (materialManager.resourceExists(MARKER_MATERIAL)) {
    return;
  }
  const auto material = materialManager.create(MARKER_MATERIAL,
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  material->setLightingEnabled(false);
  material->setDepthCheckEnabled(false);
  material->setDepthWriteEnabled(false);
  material->setCullingMode(Ogre::CULL_NONE);
  auto* pass = material->getTechnique(0)->getPass(0);
  pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  pass->setVertexColourTracking(Ogre::TVC_DIFFUSE);
  pass->setLightingEnabled(false);
}

void Detection2DArrayDisplay::reset() {
  RTDClass::reset();
  clearOverlays();
  if (textures_) {
    textures_->clear();
  }
  visualizedCount_ = 0;
  previewCount_ = 0;
}

void Detection2DArrayDisplay::clearOverlays() {
  if (markerObject_) {
    markerObject_->clear();
  }
  if (previewObject_) {
    previewObject_->clear();
  }
  for (auto& label : labels_) {
    if (label && label->text) {
      label->text->setVisible(false);
    }
  }
}

void Detection2DArrayDisplay::updateStatus() {
  size_t total = 0;
  size_t dropped = 0;
  std::string frame;
  std::string historyError;
  size_t historyCount = 0;
  size_t historySkipped = 0;
  {
    std::lock_guard<std::mutex> lock(detectionsMutex_);
    total = detections_.size();
    dropped = detections_.dropped();
    frame = frameId;
    historyError = historyError_;
    historyCount = historyLoaded_;
    historySkipped = historySkipped_;
  }
  total += historyCount;  // everything the display shows, regardless of which of the inputs brought it
  const auto imageCount = images_ ? images_->size() : 0;
  const auto showImages = showImagesProperty_ && showImagesProperty_->getBool();
  const auto imageTopic = imageTopicProperty_ ? imageTopicProperty_->getTopicStd() : std::string();
  const auto tolerance = imageToleranceProperty_ ? imageToleranceProperty_->getFloat() : 0.F;
  const auto historyEnabled = historyEnabledProperty_ && historyEnabledProperty_->getBool();
  const auto storageDir = storageDirProperty_ ? storageDirProperty_->getStdString() : std::string();

  const std::string key = frame + ';' + std::to_string(total) + ';' + std::to_string(visualizedCount_) + ';' +
                          std::to_string(poselessCount_) + ';' + std::to_string(untransformableCount_) + ';' +
                          std::to_string(previewCount_) + ';' + std::to_string(imageCount) + ';' + imageTopic +
                          ';' + std::to_string(historyCount) + ';' + std::to_string(historySkipped) + ';' +
                          std::to_string(dropped) + ';' + historyError + ';' + storageDir;
  if (key == lastStatusKey_) {
    return;
  }
  lastStatusKey_ = key;

  const auto warn = rviz_common::properties::StatusProperty::Level::Warn;
  const auto error = rviz_common::properties::StatusProperty::Level::Error;
  const auto ok = rviz_common::properties::StatusProperty::Level::Ok;

  // The persistent storage has its own status entry, so that a problem with it does not hide the state of the
  // detections received on the topic.
  if (!historyEnabled) {
    setStatusStd(ok, "Persistent history", std::string());
  } else if (!historyError.empty()) {
    setStatusStd(warn, "Persistent history", historyError);
  } else if (historySkipped > 0) {
    setStatusStd(warn, "Persistent history", "Ignored " + std::to_string(historySkipped) + " rows of the stored "
      "detections which could not be read.");
  } else {
    setStatusStd(ok, "Persistent history", "Showing " + std::to_string(historyCount) + " stored detections from " +
      storageDir + '.');
  }

  if (total == 0 && historyCount == 0) {
    setStatusStd(warn, "Detections", "Waiting for a message on the topic.");
  } else if (visualizedCount_ == 0 && poselessCount_ > 0) {
    setStatusStd(error, "3D poses", "None of the detections has a valid 3D pose in the pose of its class "
      "hypothesis, so the detections cannot be placed in the scene.");
  } else if (untransformableCount_ > 0) {
    std::string fixedFrame;
    if (context_ && context_->getFrameManager()) {
      fixedFrame = context_->getFrameManager()->getFixedFrame();
    }
    setStatusStd(warn, "Transform", "Could not transform " + std::to_string(untransformableCount_) + " of " +
      std::to_string(total) + " detections from " + globalFrameProperty_->getStdString() + " to " + fixedFrame + ".");
  } else if (untransformableGlobalCount_ > 0) {
    setStatusStd(warn, "Transform", "Could not transform " + std::to_string(untransformableGlobalCount_) + " of " +
      std::to_string(total) + " detections from " + frame + " to " + globalFrameProperty_->getStdString() + ".");
  } else if (showImages && total > 0 && imageTopic.empty()) {
    setStatusStd(warn, "Image previews",
      "No image topic is set, the detections cannot be shown with an image preview.");
  } else if (showImages && total > 0 && imageCount == 0) {
    setStatusStd(warn, "Image previews", "Waiting for images on " + imageTopic + '.');
  } else if (showImages && total > 0 && tolerance <= 0.) {
    setStatusStd(warn, "Image previews", "The image time tolerance is zero, so no image can match a detection.");
  } else if (showImages && total > 0 && !images_->lastError().empty()) {
    setStatusStd(warn, "Image previews", images_->lastError());
  } else if (showImages && total > 0 && previewCount_ == 0 && visualizedCount_ > 0) {
    setStatusStd(warn, "Image previews", "No camera image matches the detections (their timestamps differ by more "
      "than " + std::to_string(static_cast<int>(1000. * tolerance)) + " ms).");
  } else if (historyCount > 0) {
    setStatusStd(ok, "Detections", "Showing " + std::to_string(visualizedCount_) + " detections (" +
      std::to_string(historyCount) + " of them from the storage), " + std::to_string(previewCount_) +
      " of them with an image preview.");
  } else {
    auto message = "Showing " + std::to_string(visualizedCount_) + " of " + std::to_string(total) +
                   " detections, " + std::to_string(previewCount_) + " of them with an image preview.";
    if (dropped > 0) {
      message += " " + std::to_string(dropped) + " older detections were dropped (they expired or exceeded the "
                 "limit).";
    }
    setStatusStd(ok, "Detections", message);
  }
}

size_t Detection2DArrayDisplay::getVisualizationSize() const{
  return visualizedCount_;
}

size_t Detection2DArrayDisplay::getPreviewCount() const{
  return previewCount_;
}

size_t Detection2DArrayDisplay::getHistoryCount() const{
  std::lock_guard<std::mutex> lock(detectionsMutex_);
  return history_.size();
}

StorageQuery Detection2DArrayDisplay::historyQuery() const {
  const auto now = nowNs();
  StorageQuery query;
  query.rootDir = storageDirProperty_->getStdString();
  // The period is given as the age of the loaded detections, so that the shown history follows the moving now.
  query.startNs = now - 1000000000LL * static_cast<int64_t>(historyFromProperty_->getInt());
  query.endNs = now - 1000000000LL * static_cast<int64_t>(historyToProperty_->getInt());
  query.limit = static_cast<size_t>(std::max(1, historyLimitProperty_->getInt()));
  query.zoneFilter = static_cast<int>(historyZoneProperty_->getOptionInt());

  std::stringstream classes{historyClassesProperty_->getStdString()};
  std::string className;
  while (std::getline(classes, className, ',')) {
    const auto first = className.find_first_not_of(" \t");
    if (first == std::string::npos) {
      continue;
    }
    const auto last = className.find_last_not_of(" \t");
    query.classes.push_back(className.substr(first, last - first + 1));
  }
  return query;
}

void Detection2DArrayDisplay::reloadHistory() {
  historyDirty_ = true;
  if (historyEnabledProperty_ && !historyEnabledProperty_->getBool()) {
    historyDirty_ = false;
    clearHistory();
  }
  updateStatus();
  queueRender();
}

void Detection2DArrayDisplay::clearHistory() {
  {
    std::lock_guard<std::mutex> lock(detectionsMutex_);
    history_.clear();
    historyLoaded_ = 0;
    historyError_.clear();
    historySkipped_ = 0;
  }
  historyImages_.clear();
  historyImageOrder_.clear();
  updateStatus();
}

void Detection2DArrayDisplay::loadHistory() {
  historyDirty_ = false;
  historyAge_ = 0.;
  if (!historyEnabledProperty_->getBool()) {
    clearHistory();
    return;
  }
  const auto query = historyQuery();
  std::string error;
  size_t skipped = 0;
  auto records = DetectionStore::load(query, error, &skipped);
  // The global frame is the only one the stored positions are comparable in between missions, but it can be used
  // only for the detections which were recorded in the frame the user selected.
  const auto globalFrame = globalFrameProperty_ ? globalFrameProperty_->getStdString() : std::string();

  std::vector<StoredDetection> detections;
  detections.reserve(records.size());
  for (auto& record : records) {
    if (record.probability < static_cast<double>(minProbabilityProperty_->getFloat())) {
      continue;
    }
    StoredDetection stored;
    stored.stamp = rclcpp::Time(record.stampNs, RCL_ROS_TIME);
    stored.className = record.className;
    stored.probability = record.probability;
    stored.id = record.id.empty() ? record.detectionId : record.id;
    stored.history = true;
    stored.hasZone = record.zoneStatus == "in" || record.zoneStatus == "out";
    stored.inZone = record.inZone();
    stored.zoneIds = record.zoneIds;
    stored.imagePath = record.imagePath;
    if (record.imageWidth > 0 && record.imageHeight > 0) {
      // The stored image is already the crop around the detection, so it is shown as a whole.
      stored.imageRegion = {0., 0., static_cast<double>(record.imageWidth), static_cast<double>(record.imageHeight)};
    }
    record.preferGlobalFrame = !globalFrame.empty() && record.ecefFrame == globalFrame;
    double position[3];
    double orientation[4];
    record.getDisplayPose(position, orientation);
    stored.frameId = record.displayFrame();
    stored.pose.position.x = position[0];
    stored.pose.position.y = position[1];
    stored.pose.position.z = position[2];
    stored.pose.orientation.x = orientation[0];
    stored.pose.orientation.y = orientation[1];
    stored.pose.orientation.z = orientation[2];
    stored.pose.orientation.w = orientation[3];
    stored.hasPose = record.hasDisplayPose();
    detections.push_back(std::move(stored));
  }

  {
    std::lock_guard<std::mutex> lock(detectionsMutex_);
    history_ = std::move(detections);
    historyLoaded_ = history_.size();
    historyError_ = error;
    historySkipped_ = skipped;
  }
  // The images of the previous history are not shown anymore, and the images of the detections which are still shown
  // stay cached, so that a periodic reload does not have to read them again.
  updateStatus();
  queueRender();
}

const CameraImage& Detection2DArrayDisplay::historyImage(const std::string& path) {
  static const CameraImage invalid;
  if (path.empty()) {
    return invalid;
  }
  const auto cached = historyImages_.find(path);
  if (cached != historyImages_.end()) {
    return cached->second;
  }

  CameraImage image;
  const auto loaded = cv::imread(path, cv::IMREAD_UNCHANGED);
  if (!loaded.empty()) {
    // Store the image in a format which can be uploaded to a texture without further conversions.
    cv::Mat converted;
    auto encoding = sensor_msgs::image_encodings::BGR8;
    switch (loaded.channels()) {
      case 1: encoding = sensor_msgs::image_encodings::MONO8; break;
      case 4:
        cv::cvtColor(loaded, converted, cv::COLOR_BGRA2BGR);
        encoding = sensor_msgs::image_encodings::BGR8;
      break;
      default: break;
    }
    const auto& used = converted.empty() ? loaded : converted;
    image.image = std::make_shared<const cv_bridge::CvImage>(std_msgs::msg::Header(), encoding, used);
    image.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    image.scale = 1.;
  }

  // The detections are redrawn in every frame, but the images are read from the disk only once. The oldest cached
  // images are dropped when the cache would grow too large.
  historyImageOrder_.push_back(path);
  const auto inserted = historyImages_.insert({path, std::move(image)}).first;
  while (historyImageOrder_.size() > MAX_HISTORY_IMAGES) {
    const auto oldest = historyImageOrder_.begin();
    historyImages_.erase(*oldest);
    historyImageOrder_.erase(oldest);
  }
  return inserted->second;
}

namespace {

//! Maps between the 2D coordinates of the rendered view and the 3D world coordinates.
/**
 * The mapping is linear in the plane perpendicular to the viewing direction, which is exactly the plane the
 * annotations are drawn in. The mapper is created for one world point and stays valid for its neighbourhood, which is
 * where the annotations of the corresponding detection are.
 */
class ScreenMapper {
public:
  ScreenMapper(const Ogre::Camera& camera, Ogre::Viewport& viewport)
      : viewport_(&viewport) {
    const auto orientation = camera.getDerivedOrientation();
    right_ = orientation.xAxis();
    up_ = orientation.yAxis();
    forward_ = -orientation.zAxis();
    position_ = camera.getDerivedPosition();
    // Viewport::getWidth() returns the width relative to the render target, the size in pixels has a special name.
    width_ = viewport.getActualWidth();
    height_ = viewport.getActualHeight();
  }

  //! Whether the view is large enough to be used for placement.
  bool isValid() const{
    return width_ > 1. && height_ > 1.;
  }

  double width() const{
    return width_;
  }
  double height() const{
    return height_;
  }

  //! Projects the given world point to the view. Returns false if the point cannot be projected.
  bool project(const Ogre::Vector3& world, Ogre::Vector2& screen) const{
    const auto projected = rviz_rendering::project3DPointToViewportXY(viewport_, world);
    screen = {projected.x, projected.y};
    return std::isfinite(screen.x) && std::isfinite(screen.y);
  }

  //! Linearises the mapping around the given world point. The mapping is usable only if this succeeds.
  void setReferencePoint(const Ogre::Vector3& world) {
    valid_ = false;
    reference_ = world;
    if (!project(world, centre_)) {
      return;
    }

    // Measure how many pixels a small shift in the image plane is worth. Doing it numerically keeps the behaviour
    // correct for perspective as well as orthographic cameras (and for any projection RViz decides to use).
    depth_ = std::max(1e-3, static_cast<double>(std::abs((world - position_).dotProduct(forward_))));
    const auto step = std::max(1e-4, depth_ * 1e-3);

    Ogre::Vector2 shifted;
    if (!project(world + right_ * step, shifted)) {
      return;
    }
    const auto horizontal = (shifted - centre_) / step;
    if (std::abs(horizontal.x) < 1e-6) {
      return;
    }

    if (!project(world - up_ * step, shifted)) {
      return;
    }
    const auto vertical = (shifted - centre_) / step;
    if (std::abs(vertical.y) < 1e-6) {
      return;
    }

    unitX_ = right_ / horizontal.x;  // world vector of one pixel in the horizontal direction
    unitY_ = -up_ / vertical.y;  // world vector of one pixel in the vertical direction
    pixelsPerUnit_ = std::abs(horizontal.x);
    valid_ = true;
  }

  //! Whether the reference point could be projected and the linearised mapping is usable.
  bool valid() const{
    return valid_;
  }

  //! Screen position of the reference point.
  const Ogre::Vector2& centre() const{
    return centre_;
  }
  //! The reference point itself.
  const Ogre::Vector3& reference() const{
    return reference_;
  }
  //! World vector of one pixel in the horizontal direction.
  const Ogre::Vector3& unitX() const{
    return unitX_;
  }
  //! World vector of one pixel in the vertical direction.
  const Ogre::Vector3& unitY() const{
    return unitY_;
  }
  //! World vector of the given shift in view pixels.
  Ogre::Vector3 pixelOffset(double x, double y) const{
    return x * unitX_ + y * unitY_;
  }
  //! How many view pixels corresponds to one world unit in the screen plane of the reference point.
  double pixelsPerUnit() const{
    return pixelsPerUnit_;
  }

  //! How far the given screen position is from the centre of the view (0 is the centre, 1 is the border).
  double viewCentreDistance(const Ogre::Vector2& screen) const{
    const auto dx = std::abs(screen.x - width_ * .5) / width_;
    const auto dy = std::abs(screen.y - height_ * .5) / height_;
    return std::min(1., 2. * std::max(dx, dy));
  }

private:
  Ogre::Viewport* viewport_ {};  //!< Viewport the mapping corresponds to.
  Ogre::Vector3 position_ {};  //!< Position of the camera.
  Ogre::Vector3 forward_ {};  //!< Direction of the camera viewing axis.
  Ogre::Vector3 right_ {};  //!< Direction of the horizontal axis of the image.
  Ogre::Vector3 up_ {};  //!< Direction of the vertical axis of the image.
  double width_ {0};  //!< Width of the view in pixels.
  double height_ {0};  //!< Height of the view in pixels.
  Ogre::Vector3 reference_ {};  //!< The point the mapping was linearised around.
  Ogre::Vector2 centre_ {0, 0};  //!< Screen position of the reference point.
  Ogre::Vector3 unitX_ {};  //!< World vector of one pixel in the horizontal direction.
  Ogre::Vector3 unitY_ {};  //!< World vector of one pixel in the vertical direction.
  double pixelsPerUnit_ {0};  //!< How many pixels corresponds to one world unit.
  double depth_ {0};  //!< Distance of the reference point from the camera.
  bool valid_ {false};  //!< Whether the mapping is usable.
};

}  // namespace

void Detection2DArrayDisplay::update(float wall_dt, float ros_dt) {
  Display::update(wall_dt, ros_dt);
  if (!images_ || !textures_) {
    return;  // not initialized yet
  }
  // The rendering resources of the previews that are not used anymore cannot be destroyed in the callback which
  // dropped them, so do it here, in the rendering thread.
  textures_->enforceLimits();

  if (!isEnabled()) {
    clearOverlays();
    visualizedCount_ = 0;
    previewCount_ = 0;
    return;
  }

  // The detections do not arrive forever (and an empty message does not bring any), so the old ones have to be dropped
  // by their age, otherwise the scene would fill up with detections nobody sees anymore.
  {
    std::lock_guard<std::mutex> lock(detectionsMutex_);
    detections_.setLimit(static_cast<size_t>(std::max(1, maxDetectionsProperty_->getInt())));
    const auto lifetime = static_cast<double>(lifetimeProperty_->getFloat());
    if (detections_.prune(nowNs(), lifetime) > 0) {
      queueRender();
    }
  }

  // Reading the storage is much slower than drawing, so it happens only when the settings changed or when the user
  // asked for a periodic reload. When the clock is driven by the /clock topic, which has not been received yet, the
  // time range of the detections to load cannot be determined, so wait for it for a moment before guessing.
  historyAge_ += wall_dt;
  const auto reloadPeriod = historyReloadProperty_ ? historyReloadProperty_->getFloat() : 0.F;
  if (historyEnabledProperty_ && historyEnabledProperty_->getBool() &&
      (historyDirty_.load() || (reloadPeriod > 0. && historyAge_ >= static_cast<double>(reloadPeriod))))
  {
    bool clockKnown = false;
    nowNs(&clockKnown);
    if (clockKnown || historyAge_ > CLOCK_WAIT_FOR_ROS_TIME) {
      loadHistory();
    } else {
      historyDirty_ = true;
    }
  }

  // The detections have to be projected by the camera of the currently active view. The viewport it renders to tells
  // us how large the view is, which the placement of the annotations depends on.
  auto* viewManager = context_->getViewManager();
  auto* viewController = viewManager ? viewManager->getCurrent() : nullptr;
  auto* camera = viewController ? viewController->getCamera() : nullptr;
  if (!camera) {
    return;  // nothing to project the detections to
  }
  auto* renderPanel = viewManager->getRenderPanel();
  auto* renderWindow = renderPanel ? renderPanel->getRenderWindow() : nullptr;
  auto* viewport = renderWindow ? rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(renderWindow) : nullptr;
  if (!viewport) {
    viewport = camera->getViewport();
  }
  if (!viewport) {
    return;  // the view is not rendered yet
  }
  drawDetections(*camera, *viewport);
}

void Detection2DArrayDisplay::drawDetections(Ogre::Camera& camera, Ogre::Viewport& viewport) {
  clearOverlays();
  previewCount_ = 0;
  visualizedCount_ = 0;
  poselessCount_ = 0;
  untransformableCount_ = 0;

  std::vector<StoredDetection> stored;
  std::string frame;
  {
    std::lock_guard<std::mutex> lock(detectionsMutex_);
    stored.assign(detections_.detections().begin(), detections_.detections().end());
    frame = frameId;
    // The detections from the storage are older than the ones from the topic, so they are shown together with them.
    stored.insert(stored.end(), history_.begin(), history_.end());
  }
  if (stored.empty()) {
    overlayNode_->setVisible(false);
    return;
  }

  ScreenMapper mapper {camera, viewport};
  if (!mapper.isValid()) {
    return;  // the view has no sensible size yet
  }

  const auto minProbability = static_cast<double>(minProbabilityProperty_->getFloat());
  const auto markerSize = static_cast<double>(markerSizeProperty_->getFloat());
  const auto lineWidth = static_cast<double>(markerThicknessProperty_->getFloat());
  const auto fontSize = static_cast<double>(fontSizeProperty_->getFloat());
  const auto spacing = static_cast<double>(spacingProperty_->getFloat());
  const auto padding = static_cast<double>(previewPaddingProperty_->getFloat());
  const auto avoidOverlaps = avoidOverlapsProperty_->getBool();
  const auto tolerance = static_cast<double>(imageToleranceProperty_->getFloat());
  const auto showPreviews = showImagesProperty_->getBool();
  const auto useStoredImages = historyImagesProperty_->getBool();
  const auto maxPreviewWidth = static_cast<unsigned>(maxPreviewWidthProperty_->getInt());
  const auto minPreviewWidth = static_cast<unsigned>(minPreviewWidthProperty_->getInt());
  const auto maxPreviews = static_cast<size_t>(std::max(0, maxPreviewsProperty_->getInt()));
  const auto maxLabels = static_cast<size_t>(std::max(0, maxLabelsProperty_->getInt()));

  // The kept detections come from several messages and each of them has its own camera image. The detections of one
  // message share the stamp, so the image is looked up only when the stamp changes.
  const bool lookUpImages = showPreviews && tolerance > 0. && !imageTopicProperty_->getTopicStd().empty();
  rclcpp::Time lookedUpStamp {0, 0, RCL_ROS_TIME};
  CameraImage sourceImage;

  //! One detection projected to the view.
  struct Item {
    Ogre::Vector3 worldCentre {};  //!< Position of the detection.
    Ogre::Vector2 screenCentre {0, 0};  //!< Projection of the detection.
    double radius {0.};  //!< Radius of the drawn sphere (in pixels).
    Ogre::Vector3 unitX {};  //!< World vector of one pixel in the horizontal direction.
    Ogre::Vector3 unitY {};  //!< World vector of one pixel in the vertical direction.
    double pixelsPerUnit {1.};  //!< How many pixels corresponds to one world unit.
    ScreenRect anchor;  //!< Bounding box of the projected marker (in pixels).
    Ogre::ColourValue colour;  //!< Colour of the marker.
    std::string label;  //!< Text describing the detection.
    ImageRegion region;  //!< Shown part of the source image.
    CameraImage image;  //!< Image the preview is taken from.
    std::string textureKey;  //!< Identification of the preview in the texture cache.
    double aspectRatio {1.};  //!< Width / height of the preview.
    double priority {0.};  //!< Ranking of the detection among its neighbours.
    double previewWidth {0.};  //!< Requested width of the preview (in pixels).
    bool hasPreview {false};  //!< Whether an image preview is available.
  };

  auto* frameManager = context_->getFrameManager();
  const auto fixedFrame = frameManager->getFixedFrame();
  std::vector<Item> items;
  items.reserve(stored.size());
  for (const auto& detection : stored) {
    if (detection.probability < minProbability) {
      continue;
    }
    if (!detection.hasPose) {
      ++poselessCount_;
      continue;  // without a 3D pose we do not know where to draw the detection
    }
    const auto& detectionFrame = detection.frameId.empty() ? frame : detection.frameId;
    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    if (detectionFrame.empty() || detectionFrame == fixedFrame) {
      // The detection is already in the frame of the view (an empty frame means the same thing), so no transform is
      // needed (and no transform would be available when the frame of the view is not in the TF tree).
      position = toOgre(detection.pose.position);
      orientation = toOgre(detection.pose.orientation);
    } else {
      // A historical detection happened long ago and its transform is not in the TF buffer anymore, so it has to be
      // placed by the newest known transform of its frame.
      const auto detectionStamp = detection.history ? rclcpp::Time(0, 0, RCL_ROS_TIME) : detection.stamp;
      if (!frameManager->transform(detectionFrame, detectionStamp, detection.pose, position, orientation)) {
        ++untransformableCount_;
        continue;
      }
    }

    ScreenMapper local{mapper};
    local.setReferencePoint(position);
    if (!local.valid()) {
      continue;  // the detection is behind the camera
    }

    Item item;
    item.colour = getColor(detection);
    item.label = getLabel(detection);
    // The serial number is the only identification which stays the same also when the older detections are dropped
    // from the buffer, so that the preview of a detection does not have to be created again every time that happens.
    item.textureKey = std::string(detection.history ? "stored_" : "live_") + std::to_string(detection.serial);
    item.worldCentre = position;
    item.screenCentre = local.centre();
    item.unitX = local.unitX();
    item.unitY = local.unitY();
    item.pixelsPerUnit = local.pixelsPerUnit();
    item.priority = 2. * std::max(0., std::min(1., detection.probability)) +
                    1. - local.viewCentreDistance(item.screenCentre);

    // The detection is drawn as a small solid sphere at its position, the same way the PointStamped display draws a
    // point. Its size is given in the scene units, so it grows when the user zooms in; the minimum keeps the detection
    // visible also when it is far away from the camera.
    item.radius = std::max(.5 * MIN_MARKER_SIZE, .5 * markerSize * item.pixelsPerUnit);
    item.anchor = {item.screenCentre.x - item.radius, item.screenCentre.y - item.radius, 2. * item.radius,
      2. * item.radius};

    // The stored detections are shown with the image which was saved with them, the received ones with the camera
    // image they arrived in.
    if (showPreviews) {
      if (detection.history) {
        item.image = useStoredImages ? historyImage(detection.imagePath) : CameraImage{};
      } else {
        if (lookUpImages && detection.stamp != lookedUpStamp) {
          lookedUpStamp = detection.stamp;
          sourceImage = lookUpImages ? images_->get(detection.stamp, tolerance) : CameraImage{};
        }
        item.image = sourceImage;
      }
    }
    if (item.image) {
      item.hasPreview = true;
      if (detection.history) {
        item.region = detection.imageRegion;  // the stored image is already cropped around the detection
        item.aspectRatio = detection.imageRegion.width > 1. && detection.imageRegion.height > 1. ?
                             detection.imageRegion.width / detection.imageRegion.height :
                             static_cast<double>(item.image.image->image.cols) / std::max(1,
          item.image.image->image.rows);
      } else if (detection.imageRegion.width > 1. && detection.imageRegion.height > 1.) {
        item.region = detection.imageRegion.padded(.5 * (padding - 1.));
        item.aspectRatio = item.region.width / item.region.height;
      } else {
        item.region = {};  // the 2D position of the detection is not known, show the whole image
        item.aspectRatio = static_cast<double>(item.image.image->image.cols) /
                           std::max(1, item.image.image->image.rows);
      }
      // The preview is a thumbnail with a fixed size in the view. Its size must not depend on how large the detection
      // happens to be projected, otherwise zooming in would spread the detections apart but also grow their previews,
      // and zooming in would never make room for more of them.
      item.previewWidth = std::max(static_cast<double>(minPreviewWidth), static_cast<double>(maxPreviewWidth));
    }

    items.push_back(std::move(item));
  }

  visualizedCount_ = items.size();
  overlayNode_->setVisible(!items.empty());
  if (items.empty()) {
    return;
  }

  // With hundreds of detections, drawing all their labels would be slow and unreadable, so only the most important ones
  // are labelled (the same selection the previews are made from).
  std::vector<double> priorities;
  priorities.reserve(items.size());
  for (const auto& item : items) {
    if (!item.label.empty()) {
      priorities.push_back(item.priority);
    }
  }
  auto minLabelPriority = -std::numeric_limits<double>::infinity();
  if (priorities.size() > maxLabels) {
    std::nth_element(priorities.begin(), priorities.begin() + static_cast<std::ptrdiff_t>(maxLabels),
      priorities.end(), std::greater<double>());
    minLabelPriority = priorities[maxLabels];
    for (auto& item : items) {
      if (item.priority < minLabelPriority) {
        item.label.clear();
      }
    }
  }

  // Decide which detections get an image preview and where the previews are placed.
  std::vector<LayoutItem> layoutItems;
  layoutItems.reserve(items.size());
  // The annotations are identified by the index of the detection in items, which is the only identification the layout
  // planner can work with.
  for (size_t i = 0; i < items.size(); ++i) {
    const auto& item = items[i];
    LayoutItem layoutItem;
    layoutItem.id = static_cast<int>(i);
    layoutItem.anchor = {item.anchor.x - .5 * lineWidth, item.anchor.y - .5 * lineWidth,
      item.anchor.w + lineWidth, item.anchor.h + lineWidth};
    layoutItem.priority = item.priority;
    layoutItem.areaWeight = item.anchor.area();
    layoutItem.maxWidth = item.previewWidth;
    layoutItem.aspectRatio = item.aspectRatio;
    layoutItem.extraHeight = item.label.empty() ? 0. : fontSize + spacing;
    layoutItem.labelWidth = estimateLabelWidth(item.label.length(), fontSize);
    layoutItem.hasPreview = item.hasPreview;
    layoutItems.push_back(layoutItem);
  }

  std::vector<PlacedItem> placed;
  LayoutPlanner::Params params;
  params.margin = spacing;
  params.minPreviewWidth = minPreviewWidth;
  params.maxPreviews = maxPreviews;
  if (avoidOverlaps) {
    placed = LayoutPlanner::plan(layoutItems, mapper.width(), mapper.height(), params);
  } else {
    // Without the placement logic just centre every preview to the detection, whatever the consequence is.
    for (const auto& layoutItem : layoutItems) {
      if (!layoutItem.hasPreview) {
        continue;
      }
      PlacedItem placedItem;
      placedItem.id = layoutItem.id;
      placedItem.preview = true;
      placedItem.anchor = AnchorSide::CENTER;
      placedItem.rect = anchoredRect(layoutItem.anchor, AnchorSide::CENTER, layoutItem.maxWidth,
          layoutItem.maxWidth / std::max(1e-3, layoutItem.aspectRatio), params.margin);
      placedItem.labelRect = {placedItem.rect.x, placedItem.rect.y + placedItem.rect.h, placedItem.rect.w,
        layoutItem.extraHeight};
      placed.push_back(placedItem);
    }
  }
  std::map<unsigned, const PlacedItem*> annotations;
  for (const auto& placedItem : placed) {
    if (placedItem.id >= 0) {
      annotations[static_cast<unsigned>(placedItem.id)] = &placedItem;
    }
  }

  // The markers are drawn as quads in the plane of the screen, so that their size can be set in pixels.
  auto vertexCount = 0u;

  // The sphere is drawn as a filled disc in the plane of the screen, with a darker outline and a light spot, so that
  // it looks like the shaded sphere the PointStamped display draws, but without the cost of a 3D mesh per detection.
  auto addDisc = [this, &vertexCount](const ScreenMapper& local, const Ogre::Vector3& centre, double radius,
                                      const Ogre::ColourValue& colour) {
                   if (!(radius > 0.)) {
                     return;
                   }
                   const auto base = vertexCount;
                   vertexCount += SPHERE_SEGMENTS + 1;
                   markerObject_->position(centre);
                   markerObject_->colour(colour);
                   for (unsigned segment = 0; segment < SPHERE_SEGMENTS; ++segment) {
                     const auto angle = 2. * M_PI * static_cast<double>(segment) /
                                        static_cast<double>(SPHERE_SEGMENTS);
                     markerObject_->position(centre + local.pixelOffset(radius * std::cos(angle),
                       radius * std::sin(angle)));
                     markerObject_->colour(colour);
                   }
                   for (unsigned segment = 1; segment + 1 <= SPHERE_SEGMENTS; ++segment) {
                     markerObject_->triangle(base, base + segment, base + segment + 1);
                   }
                 };

  markerObject_->begin(MARKER_MATERIAL, Ogre::RenderOperation::OT_TRIANGLE_LIST);
  vertexCount = 0;
  for (const auto& item : items) {
    ScreenMapper local{mapper};
    local.setReferencePoint(item.worldCentre);
    if (!local.valid()) {
      continue;
    }
    if (lineWidth > 0.) {
      addDisc(local, item.worldCentre, item.radius + .5 * lineWidth,
        Ogre::ColourValue{.15F * item.colour.r, .15F * item.colour.g, .15F * item.colour.b, item.colour.a});
    }
    addDisc(local, item.worldCentre, item.radius, item.colour);
    if (item.radius > 3. && items.size() <= SHADED_MARKER_LIMIT) {
      // The light spot makes the flat disc look like a sphere lit from the front upper left side.
      addDisc(local, item.worldCentre + local.pixelOffset(-.25 * item.radius, -.25 * item.radius),
        .3 * item.radius, Ogre::ColourValue{std::min(1.F, .5F + .5F * item.colour.r),
            std::min(1.F, .5F + .5F * item.colour.g), std::min(1.F, .5F + .5F * item.colour.b), .6F * item.colour.a});
    }
  }
  markerObject_->end();

  // The previews are quads in the plane of the screen, so they always face the camera. Their textures are cropped from
  // the source camera image, so that the preview shows exactly what the detector saw.
  if (!annotations.empty()) {
    std::map<std::string, std::vector<std::pair<const Item*, ScreenRect>>> byMaterial;
    for (const auto& [index, placedItem] : annotations) {
      if (index >= items.size() || !placedItem->preview) {
        continue;
      }
      const auto& item = items[index];
      unsigned textureWidth = 0;
      unsigned textureHeight = 0;
      const auto requestedWidth = static_cast<unsigned>(std::max(1., static_cast<double>(placedItem->rect.w)));
      const auto material = textures_->getMaterial(item.image, item.region, requestedWidth, item.textureKey,
          textureWidth, textureHeight);
      if (material.empty()) {
        continue;
      }
      byMaterial[material].emplace_back(&item, placedItem->rect);
      ++previewCount_;
    }
    for (const auto& [material, materialItems] : byMaterial) {
      previewObject_->begin(material, Ogre::RenderOperation::OT_TRIANGLE_LIST);
      vertexCount = 0;
      for (const auto& [item, rect] : materialItems) {
        // The corners are computed in the plane of the screen of the detection, so the preview always faces the
        // camera and has the same scale as the detection next to it.
        const Ogre::Vector2 corners[4] = {
          {static_cast<float>(rect.x), static_cast<float>(rect.y)},
          {static_cast<float>(rect.x + rect.w), static_cast<float>(rect.y)},
          {static_cast<float>(rect.x + rect.w), static_cast<float>(rect.y + rect.h)},
          {static_cast<float>(rect.x), static_cast<float>(rect.y + rect.h)}};
        const Ogre::Vector2 textureCoords[4] = {{0., 0.}, {1., 0.}, {1., 1.}, {0., 1.}};
        const Ogre::ColourValue white{1., 1., 1., 1.};
        const auto base = vertexCount;
        vertexCount += 4;
        for (int corner = 0; corner < 4; ++corner) {
          previewObject_->position(item->worldCentre + (corners[corner].x - item->screenCentre.x) * item->unitX +
              (corners[corner].y - item->screenCentre.y) * item->unitY);
          previewObject_->textureCoord(textureCoords[corner].x, textureCoords[corner].y);
          previewObject_->colour(white);
        }
        previewObject_->triangle(base, base + 1, base + 2);
        previewObject_->triangle(base, base + 2, base + 3);
      }
      previewObject_->end();
    }
  }

  // The text labels are drawn under the annotation of the detection they belong to.
  if (!textFailed_) {
    size_t labelIndex = 0;
    auto addLabel = [this, &labelIndex](const std::string& text, const Ogre::Vector3& position,
                                        double height, const Ogre::ColourValue& colour) {
                      if (labelIndex >= labels_.size()) {
                        try {
                          auto label = std::make_unique<Label>();
                          label->text = std::make_unique<rviz_rendering::MovableText>(text,
              fontProperty_->getString().toStdString(), height);
                          label->text->setQueryFlags(0);  // the annotations are not pickable
                          label->text->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);
                          label->text->setTextAlignment(rviz_rendering::MovableText::H_LEFT,
            rviz_rendering::MovableText::V_ABOVE);
                          overlayNode_->attachObject(label->text.get());
                          labels_.push_back(std::move(label));
                        } catch (const Ogre::Exception& e) {
                          textFailed_ = true;
                          RCLCPP_WARN_STREAM(rclcpp::get_logger("cras_rviz_vision_msgs.Detection2DArrayDisplay"),
            "Could not render the detection labels: " << e.what());
                          return;
                        }
                      }
                      auto& drawnText = *labels_[labelIndex]->text;
                      drawnText.setVisible(true);
                      drawnText.setCaption(text);
                      drawnText.setCharacterHeight(height);
                      drawnText.setColor(colour);
                      drawnText.setGlobalTranslation(position);
                      ++labelIndex;
                    };
    for (size_t i = 0; i < items.size(); ++i) {
      const auto& item = items[i];
      if (item.label.empty()) {
        continue;
      }
      const auto annotation = annotations.find(static_cast<unsigned>(i));
      ScreenRect labelRect;
      if (annotation != annotations.end() && annotation->second->labelRect.area() > 0.) {
        labelRect = annotation->second->labelRect;
      } else if (!avoidOverlaps) {
        // Without the placement logic just put the label under the detection, whatever it lands on.
        ScreenRect anchor = item.anchor;
        if (annotation != annotations.end() && annotation->second->preview) {
          anchor = annotation->second->rect;
        }
        labelRect = {anchor.x, anchor.y + anchor.h + spacing, 1., 1.};
      } else {
        // The placement did not find any free place for this label, so rather do not show it than to make the crowd of
        // the overlapping texts unreadable.
        continue;
      }
      const auto position = item.worldCentre + (labelRect.x - item.screenCentre.x) * item.unitX +
                            (labelRect.y - item.screenCentre.y) * item.unitY;
      addLabel(item.label, position, fontSize / std::max(1e-6, item.pixelsPerUnit), item.colour);
    }
  }

  updateStatus();
}

}  // namespace cras_rviz_vision_msgs

PLUGINLIB_EXPORT_CLASS(cras_rviz_vision_msgs::Detection2DArrayDisplay, rviz_common::Display)
