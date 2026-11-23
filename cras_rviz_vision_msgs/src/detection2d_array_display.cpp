// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Implementation of the visualization of vision_msgs/Detection2DArray messages.
 */

#include "cras_rviz_vision_msgs/detection2d_array_display.hpp"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <functional>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <OgreCamera.h>
#include <OgreEntity.h>
#include <OgreException.h>
#include <OgreMaterial.h>
#include <OgreMaterialManager.h>
#include <OgreQuaternion.h>
#include <OgreSceneManager.h>
#include <OgreTechnique.h>
#include <OgreViewport.h>
#include <QEvent>
#include <QMouseEvent>
#include <QWindow>
#include <rclcpp/logging.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/transformation/frame_transformer.hpp>
#include <rviz_common/view_controller.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_rendering/geometry.hpp>
#include <rviz_rendering/material_manager.hpp>
#include <rviz_rendering/render_window.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/time.h>

namespace cras_rviz_vision_msgs {

namespace {

//! Maximal size of the cached camera images (in pixels); the previews never need the original resolution.
constexpr size_t MAX_IMAGE_SIZE = 640;
//! How many camera images are kept for matching the detections to them (they are kept downscaled, so this is cheap).
constexpr size_t MAX_CACHED_IMAGES = 64;
//! How many images of the stored detections are kept in memory.
constexpr size_t MAX_HISTORY_IMAGES = 64;
//! A detection marker is never drawn smaller than this, so that it does not disappear when it is far away.
constexpr double MIN_MARKER_SIZE = 3.;  // [px]
//! How many detections are considered for the available preview slots (the rest does not have its image prepared).
constexpr size_t previewCandidateLimit(size_t maxPreviews) {
  return 2 * maxPreviews + 8;
}
//! Radius (in pixels) around a marker which the mouse still points at it.
constexpr double HOVER_RADIUS = 8.;
//! Smallest radius (in pixels) of a marker the mouse can still point at, however small the marker is drawn.
constexpr double MIN_HIT_RADIUS = 14.;
//! How much bigger than its picture an image preview is clickable [px]; the frame and the line to the marker are there.
constexpr double PREVIEW_MARGIN = 6.;
//! How often to look again for a transform which was not available. [s]
constexpr double RETRY_TRANSFORMS_PERIOD = .5;
//! A detection older than this (and not transformed yet) will never get its transform from the TF buffer anymore. [s]
constexpr double MAX_TRANSFORM_AGE = 600.;
//! How often the informational status messages are repeated in the log when the display logs its status. [s]
constexpr double STATUS_LOG_PERIOD = 5.;

//! A time difference in words a person can read quickly: only the first significant digit and its neighbour.
double coarseTime(double nanoseconds) {
  const auto seconds = std::abs(nanoseconds) / 1e9;
  if (seconds < 1.) {
    return std::round(std::abs(nanoseconds) / 1e6) * 1e-3;  // the whole number is in the milliseconds already
  }
  if (seconds < 60.) {
    return std::round(seconds * 10.) * .1;
  }
  return std::round(seconds / 60.) * 60.;
}

//! The same for a distance in pixels.
double coarseDistance(double pixels) {
  if (pixels < 10.) {
    return std::round(pixels);
  }
  return std::round(pixels / 10.) * 10.;
}

//! A time difference in the status message, in milliseconds or seconds, whichever is easier to read.
std::string formatTime(double nanoseconds) {
  const auto seconds = coarseTime(nanoseconds);
  if (seconds < 10.) {
    return std::to_string(static_cast<int>(std::llround(seconds * 1000.))) + " ms";
  }
  return std::to_string(static_cast<int>(std::llround(seconds))) + " s";
}

//! Mesh of the detection markers; the same sphere the PointStamped display and rviz_rendering::Shape draw.
constexpr const char* const SPHERE_MESH = "rviz_sphere.mesh";
//! Where the marker of a highlighted detection is drawn: over all the normal scene content, under the annotations.
constexpr unsigned HIGHLIGHT_QUEUE = Ogre::RENDER_QUEUE_OVERLAY - 1;
//! How long a highlight is looked for again in the view, so that it ends on time even when nothing moves [s].
constexpr double SELECTION_REFRESH_PERIOD = .25;
//! How far the mouse can move between the press and the release for it to still count as a click [px].
constexpr double CLICK_TOLERANCE = 5.;
//! How many detections can be waiting for their highlight at once (the one clicked last is shown).
constexpr size_t MAX_REQUESTED_DETECTIONS = 16;

//! Time of the transform which is the newest one available (RViz interprets the zero time that way).
const rclcpp::Time& latestTime() {
  static const rclcpp::Time time {0, 0, RCL_ROS_TIME};
  return time;
}

//! Converts the quaternion of the given ROS orientation to an Ogre quaternion.
inline Ogre::Quaternion toOgre(const geometry_msgs::msg::Quaternion& orientation) {
  return Ogre::Quaternion(orientation.w, orientation.x, orientation.y, orientation.z);
}

//! Converts the given ROS time to a tf2 time.
tf2::TimePoint toTfTime(const rclcpp::Time& time) {
  return tf2::TimePoint {std::chrono::duration_cast<tf2::Duration>(std::chrono::nanoseconds(
          time.nanoseconds()))};
}

/**
 * \brief Transform of \c frame into \c target at the given time.
 *
 * Unlike the transform methods of the RViz frame manager, this one does not fall back to the newest available transform
 * when the requested time is not covered by the TF buffer. That would silently place an old detection somewhere else,
 * and it would move around as the frames move, so the display has to know that the transform is not available.
 *
 * \param[in] transformer The transform manager of the RViz instance (may be null).
 * \param[in] target Frame to transform the pose to.
 * \param[in] frame Frame the pose is in.
 * \param[in] time When the transform is looked for. It never waits for anything.
 * \return The transform, or nothing when it is not available.
 */
std::optional<tf2::Transform> lookupTransform(
    const std::shared_ptr<rviz_common::transformation::FrameTransformer>& transformer,
    const std::string& target,
    const std::string& frame, const rclcpp::Time& time) {
  if (!transformer || target.empty() || frame.empty()) {
    return std::nullopt;
  }
  if (target == frame) {
    return tf2::Transform::getIdentity();
  }
  try {
    const auto transform = transformer->lookupTransform(target, frame, toTfTime(time)).transform;
    return tf2::Transform {tf2::Quaternion {transform.rotation.x, transform.rotation.y,
        transform.rotation.z,
        transform.rotation.w}, tf2::Vector3 {transform.translation.x, transform.translation.y,
        transform.translation.z}};
  } catch (const std::exception&) {
    return std::nullopt;  // the frames are not connected, or the time is not covered by the transform buffer
  }
}

/**
 * \brief Values of a comma-separated list, with the whitespace around them removed.
 *
 * Used by the properties where the user lists several classes or camera frames.
 */
std::vector<std::string> splitList(const std::string& text) {
  std::vector<std::string> values;
  std::stringstream list{text};
  std::string value;
  while (std::getline(list, value, ',')) {
    const auto first = value.find_first_not_of(" \t");
    if (first == std::string::npos) {
      continue;
    }
    const auto last = value.find_last_not_of(" \t");
    values.push_back(value.substr(first, last - first + 1));
  }
  return values;
}

//! Transforms the given pose (given in the frame the transform is the target frame of) by the given transform.
void applyTransform(const tf2::Transform& transform, geometry_msgs::msg::Pose& pose) {
  const auto position = transform * tf2::Vector3(pose.position.x, pose.position.y, pose.position.z);
  const auto orientation = transform.getRotation() *
                           tf2::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z,
      pose.orientation.w);
  pose.position.x = position.x();
  pose.position.y = position.y();
  pose.position.z = position.z();
  pose.orientation.x = orientation.x();
  pose.orientation.y = orientation.y();
  pose.orientation.z = orientation.z();
  pose.orientation.w = orientation.w();
}

//! Converts the given colour from the HSV space to the RGB space.
inline Ogre::ColourValue hsvToColour(float hue, float saturation, float value) {
  const auto component = [hue, saturation, value](int n) {
                           const auto k = std::fmod(static_cast<float>(n) + 4.f * hue, 6.f);
                           return value - value * saturation * std::min(std::max(k, 0.f), std::min(4.f - k, 1.f));
                         };
  return Ogre::ColourValue(component(5), component(3), component(1), 1.f);
}

//! Formats the moment of a detection as the clock time the user knows it by (hour and minute).
std::string formatClock(int64_t stampNs) {
  const auto seconds = static_cast<time_t>(stampNs / 1000000000LL);
  std::tm moment{};
  if (localtime_r(&seconds, &moment) == nullptr) {
    return std::to_string(seconds);
  }
  char buffer[16];
  if (std::strftime(buffer, sizeof(buffer), "%H:%M", &moment) == 0) {
    return std::to_string(seconds);
  }
  return buffer;
}

//! Replaces everything but letters and digits in the given name, so that it can be used in a resource name.
std::string sanitize(const std::string& name) {
  std::string clean;
  for (const auto character : name) {
    if (std::isalnum(static_cast<unsigned char>(character))) {
      clean += character;
    }
  }
  return clean.empty() ? "rviz" : clean;
}

/**
 * \brief Maps between the 2D coordinates of the rendered view and the 3D world coordinates.
 *
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
    screen = Ogre::Vector2(projected.x, projected.y);
    return std::isfinite(screen.x) && std::isfinite(screen.y);
  }

  //! Linearises the mapping around the given world point. The mapping is usable only if this succeeds.
  void setReferencePoint(const Ogre::Vector3& world) {
    valid_ = false;
    if (!project(world, centre_)) {
      return;
    }

    // Measure how many pixels a small shift in the image plane is worth. Doing it numerically keeps the behaviour
    // correct for perspective as well as orthographic cameras (and for any projection RViz decides to use).
    const auto depth = std::max(1e-3,
          static_cast<double>(std::abs((world - position_).dotProduct(forward_))));
    const auto step = std::max(1e-4, depth * 1e-3);

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
  //! World vector of one pixel in the horizontal direction.
  const Ogre::Vector3& unitX() const{
    return unitX_;
  }
  //! World vector of one pixel in the vertical direction.
  const Ogre::Vector3& unitY() const{
    return unitY_;
  }
  //! How many view pixels corresponds to one world unit in the screen plane of the reference point.
  double pixelsPerUnit() const{
    return pixelsPerUnit_;
  }

  //! Whether the given screen position is inside the view at all.
  bool insideView(const Ogre::Vector2& screen) const{
    return screen.x >= 0. && screen.y >= 0. && screen.x <= width_ && screen.y <= height_;
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
  Ogre::Vector2 centre_ {0, 0};  //!< Screen position of the reference point.
  Ogre::Vector3 unitX_ {};  //!< World vector of one pixel in the horizontal direction.
  Ogre::Vector3 unitY_ {};  //!< World vector of one pixel in the vertical direction.
  double pixelsPerUnit_ {0};  //!< How many pixels corresponds to one world unit.
  bool valid_ {false};  //!< Whether the mapping is usable.
};

}  // namespace

/**
 * \brief Watches the mouse movement in the view so that the detection the user points at can be highlighted.
 *
 * The display cannot get the mouse events from RViz (they all belong to the active tool), but it can listen on the
 * rendering window itself. The events are only watched, never consumed, so that the tools keep working as before.
 */
class MouseWatcher : public QObject {
public:
  explicit MouseWatcher(Detection2DArrayDisplay& display)
      : display_(display) {
  }

  bool eventFilter(QObject* watched, QEvent* event) override;

private:
  Detection2DArrayDisplay& display_;
};

bool MouseWatcher::eventFilter(QObject* watched, QEvent* event) {
  switch (event->type()) {
    case QEvent::MouseMove:
    {
        // The rendering window is a QWindow embedded in the RViz widget, so its coordinates are the window ones, not
        // the pixels of the viewport the detections are projected to.
      const auto* mouse = static_cast<QMouseEvent*>(event);
      display_.mouseX_ = mouse->localPos().x() * display_.mouseScaleX_;
      display_.mouseY_ = mouse->localPos().y() * display_.mouseScaleY_;
      display_.mouseInside_ = true;
        // The hovered detection can change even when the layout stays the same, so the detections have to be drawn
        // again to notice it.
      display_.layoutDirty_ = true;
      display_.queueRender();
      break;
    }
    case QEvent::Leave:
      if (display_.mouseInside_) {
        display_.mouseInside_ = false;
        display_.layoutDirty_ = true;
        display_.queueRender();
      }
    break;
    case QEvent::MouseButtonPress:
    case QEvent::MouseButtonRelease:
    {
      // A click is a press and a release at the same place; when the mouse moved in between, the user was using the
      // tool of the view (rotating the scene, moving an object) and did not mean to highlight anything.
      const auto* mouse = static_cast<QMouseEvent*>(event);
      if (mouse->button() != Qt::LeftButton && mouse->button() != Qt::NoButton) {
        break;  // the other buttons belong to the tools of the view
      }
      const auto x = mouse->localPos().x() * display_.mouseScaleX_;
      const auto y = mouse->localPos().y() * display_.mouseScaleY_;
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("cras_rviz_vision_msgs.Detection2DArrayDisplay"),
        "Mouse " << (event->type() ==
                     QEvent::MouseButtonPress ? "pressed" : "released") << " at " << x << ", " << y <<
            " of the view.");
      if (event->type() == QEvent::MouseButtonPress) {
        display_.mousePressed(x, y);
      } else {
        display_.mouseReleased(x, y);
      }
      break;
    }
    default:
    break;
  }
  return false;  // the events belong to the active tool, we only look at them
}

Detection2DArrayDisplay::Detection2DArrayDisplay() {
  imageTopicProperty_ = new ImageTopicProperty("Image topic",
      "Camera images the detections come from. The part of the image around the detection bounding box is shown as "
      "the detection preview. Both sensor_msgs/msg/Image and sensor_msgs/msg/CompressedImage topics can be used.",
      this, SLOT(updateImageTopic()));

  imageToleranceProperty_ = new rviz_common::properties::FloatProperty("Image time tolerance", .5,
      "Maximal time difference between the detection message and the camera image to show the image preview [s]. The "
      "images are often published with a delay, so the image matching a detection can arrive only some time after the "
      "detection itself - the preview then appears later, but it stays. Set this bigger if your camera pipeline is "
      "slow; the status of the display tells how far the images really are.", this);
  imageToleranceProperty_->setMin(0.);
  imageToleranceProperty_->setMax(60.);

  updatePeriodProperty_ = new rviz_common::properties::FloatProperty("Layout update period", 1.,
      "How often the markers, labels and image previews can be rearranged [s]. The detections arrive much faster, and "
      "following every message would make the previews jump around and the view flicker. The status of the display "
      "follows the changes immediately; set 0 to redraw with every change.", this);
  updatePeriodProperty_->setMin(0.);
  updatePeriodProperty_->setMax(60.);

  statusDebugProperty_ = new rviz_common::properties::BoolProperty("Log status", false,
      "Report every change of the status of this display to the ROS log. Useful when the panel of the display is not "
      "visible, e.g. when RViz runs inside another application or in the background. The messages which are only "
      "informational are reported at most once per few seconds to not flood the log.", this);

  auto* markerCategory = new rviz_common::properties::Property("Markers", "",
      "How the detections themselves are drawn.", this);
  maxMarkersProperty_ = new rviz_common::properties::IntProperty("Max. markers", 2000,
      "Maximal number of detections shown at once. When more of them is remembered, the oldest ones are dropped.",
      markerCategory);
  maxMarkersProperty_->setMin(1);
  maxMarkersProperty_->setMax(1000000);
  markerSizeProperty_ = new rviz_common::properties::FloatProperty("Size", .1,
      "Size of the drawn sphere [m]. It is drawn the same way the PointStamped display draws a point, i.e. with a size "
      "given in the scene units, so it grows when the user zooms in.", markerCategory);
  markerSizeProperty_->setMin(0.001);
  markerSizeProperty_->setMax(100.);
  accumulateProperty_ = new rviz_common::properties::BoolProperty("Keep older detections", true,
      "Whether the detections from the older messages should stay visible. When enabled, an empty message does not "
      "hide the detections and the display can show more detections than a single message contains.",
      markerCategory);
  lifetimeProperty_ = new rviz_common::properties::FloatProperty("Max. age out of a zone", 0.,
      "How long a detection which is not reported again and which is out of every keep-out zone stays shown, in "
      "seconds. 0 keeps it until the number above is exceeded. The detections loaded from the persistent storage age "
      "the same way, counted from the moment they were recorded.", markerCategory);
  lifetimeProperty_->setMin(0.);
  lifetimeProperty_->setMax(86400.);
  inZoneLifetimeProperty_ = new rviz_common::properties::FloatProperty("Max. age in a zone", 0.,
      "How long a detection which is not reported again and which is inside a keep-out zone stays shown, in seconds. "
      "Objects which got into a zone are usually the interesting ones, so they can stay for a different time than the "
      "ones out of all the zones. 0 keeps them until the number above is exceeded. It applies to the stored detections "
      "whose place lies in a zone now too.", markerCategory);
  inZoneLifetimeProperty_->setMin(0.);
  inZoneLifetimeProperty_->setMax(86400.);
  zonesTopicProperty_ = new rviz_common::properties::RosTopicProperty("Keep-out zones topic", "polygons",
      rosidl_generator_traits::name<polygon_msgs::msg::Polygon2DCollection>(),
      "The topic the keep-out zones (the polygons drawn by the zone of interest tool) are published on. The display "
      "needs them to tell which detections are inside a zone, both to colour them and to let them expire after a "
      "different time.", markerCategory, SLOT(updateZonesTopic()), this);
  minProbabilityProperty_ = new rviz_common::properties::FloatProperty("Min. probability", 0.,
      "Detections with a lower probability than this are not visualized.", markerCategory);
  minProbabilityProperty_->setMin(0.);
  minProbabilityProperty_->setMax(1.);
  colorModeProperty_ = new rviz_common::properties::EnumProperty("Color by", "Probability",
      "How the colour of the detection marker is chosen.", markerCategory);
  colorModeProperty_->addOption("Probability", static_cast<int>(ColorMode::PROBABILITY));
  colorModeProperty_->addOption("Class name", static_cast<int>(ColorMode::CLASS_NAME));
  colorModeProperty_->addOption("Constant", static_cast<int>(ColorMode::NONE));
  colorModeProperty_->addOption("Keep-out zone", static_cast<int>(ColorMode::ZONE));

  auto* mergeCategory = new rviz_common::properties::Property("Merge similar detections", "",
      "A detection which is close to an already remembered one replaces it instead of being added, so that the "
      "remembered detections stay sparse. All the thresholds below have to be satisfied for a detection to be "
      "replaced.", this);
  merge3dProperty_ = new rviz_common::properties::FloatProperty("Distance", 1.,
      "Maximal distance of the 3D positions of two detections for them to be the same object [m]. Set 0 to remember "
      "every detection separately.", mergeCategory);
  merge3dProperty_->setMin(0.);
  merge3dProperty_->setMax(100.);
  merge2dProperty_ = new rviz_common::properties::FloatProperty("Image distance", 50.,
      "Maximal distance of the centres of the bounding boxes of two detections in the source image [px]. Set 0 to "
      "ignore the image position.", mergeCategory);
  merge2dProperty_->setMin(0.);
  merge2dProperty_->setMax(2000.);
  mergeTimeProperty_ = new rviz_common::properties::FloatProperty("Time distance", 5.,
      "Maximal difference of the timestamps of two detections for them to be the same object [s]. Set 0 to ignore the "
      "time.", mergeCategory);
  mergeTimeProperty_->setMin(0.);
  mergeTimeProperty_->setMax(3600.);

  auto* labelCategory = new rviz_common::properties::Property("Labels", "",
      "Text describing each detection.", this);
  showLabelsProperty_ = new rviz_common::properties::BoolProperty("Show labels", true,
      "Whether the text labels next to the detections should be shown.", labelCategory);
  showClassProperty_ = new rviz_common::properties::BoolProperty("Show class", true,
      "Whether the name of the most probable class should be part of the label.", labelCategory);
  showProbabilityProperty_ = new rviz_common::properties::BoolProperty("Show probability", true,
      "Whether the probability of the most probable class should be part of the label.",
      labelCategory);
  showIdProperty_ = new rviz_common::properties::BoolProperty("Show ID", false,
      "Whether the ID of the detection should be part of the label.", labelCategory);
  showTimeProperty_ = new rviz_common::properties::BoolProperty("Show time", true,
      "Whether the hour and minute of the moment a detection was made should be part of its label. It tells the "
      "detections which stay in the view for a long time apart.", labelCategory);
  showZoneProperty_ = new rviz_common::properties::BoolProperty("Show zone", true,
      "Whether the keep-out zone the detection is in (or the information that it is in none) should be part of the "
      "label.", labelCategory);
  maxLabelsProperty_ = new rviz_common::properties::IntProperty("Max. labels", 200,
      "Maximal number of labels which are shown at once. With hundreds of detections, drawing all the labels would be "
      "slow and unreadable, so only the labels of the most important detections are shown.",
      labelCategory);
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
  maxPreviewsProperty_ = new rviz_common::properties::IntProperty("Max. images", 10,
      "Maximal number of detections shown with an image preview at once. Each preview needs a texture in the graphics "
      "memory, and the layout has to look for room for it, so showing all of them in a scene with a lot of detections "
      "would be slow and unreadable.", imageCategory);
  maxPreviewsProperty_->setMin(0);
  maxPreviewsProperty_->setMax(10000);
  previewWidthProperty_ = new rviz_common::properties::IntProperty("Preview width", 150,
      "Width of an image preview in the view [px]. Its height follows the part of the camera image which the preview "
      "shows, so the picture of a tall detection and of a wide one have a different shape - an image is never "
      "stretched. The previews do not grow or shrink with the zooming, so zooming in makes room for more of them.",
      imageCategory);
  previewWidthProperty_->setMin(16);
  previewWidthProperty_->setMax(1024);
  previewPaddingProperty_ = new rviz_common::properties::FloatProperty("Preview padding", 1.2,
      "The preview shows a part of the image this much bigger than the detection bounding box. The visualization node"
      " of the detections uses the same rule (its bbox_scale parameter), so that the preview of a detection shows the"
      " same cutout as the picture of the object the user clicks on in the visualization strip.",
      imageCategory);
  previewPaddingProperty_->setMin(1.);
  previewPaddingProperty_->setMax(10.);
  spacingProperty_ = new rviz_common::properties::FloatProperty("Spacing", 6.,
      "Minimal spacing between two annotations [px].", imageCategory);
  spacingProperty_->setMin(0.);
  spacingProperty_->setMax(50.);
  stemProperty_ = new rviz_common::properties::FloatProperty("Connector width", 2.,
      "Width of the line which connects an image preview to the marker of its detection [px]. The layout keeps the "
      "line free of the other annotations, so the previews stay clearly attributable to their detections. Set 0 to "
      "draw no lines.", imageCategory);
  stemProperty_->setMin(0.);
  stemProperty_->setMax(20.);
  previewAlphaProperty_ = new rviz_common::properties::FloatProperty("Preview alpha", .75,
      "How opaque the image previews are. The hovered and the highlighted preview are always fully opaque.",
      imageCategory);
  previewAlphaProperty_->setMin(.1);
  previewAlphaProperty_->setMax(1.);
  previewFrameProperty_ = new rviz_common::properties::BoolProperty("Preview frame", true,
      "Whether to draw a coloured frame around each image preview. The frame has the colour of the detection and the "
      "frame of the hovered preview is brighter, which is what tells the previews apart when they are close to each "
      "other.", imageCategory);
  cameraFrameProperty_ = new rviz_common::properties::StringProperty("Camera frame", "",
      "Which camera to take the preview images from. Leave empty to take them from the camera the detection was "
      "reported by (its message frame), which is what to do when several cameras publish their detections and their "
      "images to one topic each or to a shared topic. Set the name of a camera frame to always use that one, or * to "
      "use the closest image whatever camera it comes from.", imageCategory);
  hoverDelayProperty_ = new rviz_common::properties::FloatProperty("Hover delay", .4,
      "How long the mouse has to stay over a detection for its image preview to be shown. Shows the image of the "
      "detection the user looks at, without the previews jumping around when the mouse only passes over them.",
      imageCategory);
  hoverDelayProperty_->setMin(0.);
  hoverDelayProperty_->setMax(10.);

  auto* selectionCategory = new rviz_common::properties::Property("Selection", "",
      "What happens when the user clicks on a detection, either in the view of this display, or in a camera image "
      "shown by another part of the interface (e.g. rqt_image_view).", this);
  // The name of the topic the clicks in a camera image arrive on. It cannot be offered in a list of topics because the
  // display does not know which of the two message types the user wants to publish there.
  selectionTopicNameProperty_ = new rviz_common::properties::StringProperty("Clicked image topic",
      "/clicked_image",
      "Topic to listen on for the places the user clicks in a camera image. Both the detection visualizer of the "
      "g4s_mission package and the mouse click publisher of rqt_image_view publish there.",
      selectionCategory);
  selectionTopicProperty_ = new rviz_common::properties::EnumProperty("Clicked image type", "Point",
      "Which message announces that the user clicked somewhere in a camera image. The topic is subscribed to with that "
      "message type, so it has to be the type the publisher really publishes. The detector visualizer of the "
      "g4s_mission package publishes both of them (the Point message on the topic this property offers by default). "
      "The Point message tells where in the image the click was, the Header message only says which image it was (the "
      "most recent detection of that camera is highlighted then).", selectionCategory);
  selectionTopicProperty_->addOption("Header", static_cast<int>(SelectionMode::HEADER));
  selectionTopicProperty_->addOption("Point", static_cast<int>(SelectionMode::POINT));
  selectionTopicProperty_->addOption("Nothing", static_cast<int>(SelectionMode::NONE));
  selectionToleranceProperty_ = new rviz_common::properties::FloatProperty("Image time tolerance",
      2.,
      "How far apart the clicked camera image and a detection can be in time for the detection to be highlighted. "
      "Detections are usually a little older than the image they are shown in.", selectionCategory);
  selectionToleranceProperty_->setMin(0.);
  selectionToleranceProperty_->setMax(60.);
  highlightProperty_ = new rviz_common::properties::FloatProperty("Highlight", 30.,
      "How long a clicked detection stays highlighted, in seconds. Clicking it again starts the time again.",
      selectionCategory);
  highlightProperty_->setMin(0.);
  highlightProperty_->setMax(3600.);
  highlightSizeProperty_ = new rviz_common::properties::FloatProperty("Marker scale", 2.,
      "How much bigger the marker of a highlighted detection is than the markers of the other ones.",
      selectionCategory);
  highlightSizeProperty_->setMin(1.);
  highlightSizeProperty_->setMax(10.);
  highlightColorProperty_ = new rviz_common::properties::ColorProperty("Color", QColor(0, 200, 0),
      "Colour of the frame of the highlighted image preview. The marker of a highlighted detection is drawn in that "
      "colour too, and above all the other markers.", selectionCategory);

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
      "Frame the detections keep their global (Earth-centred) position in, typically FP_ECEF. The detections are "
      "remembered in it because the transform from it to the frame of the view is static and stays valid for any age "
      "of the detections. Leave empty to remember them in the frame they arrived in.",
      historyCategory);
  historyFromProperty_ = new rviz_common::properties::IntProperty("Load from (s ago)", 0,
      "Age of the oldest detection to load [s]. 0 loads the whole storage whatever its age is, which is what you want "
      "when replaying an older recording (the stored stamps are absolute, so a limit relative to the current clock "
      "would hide them).", historyCategory);
  historyFromProperty_->setMin(0);
  historyFromProperty_->setMax(365 * 24 * 3600);
  historyToProperty_ = new rviz_common::properties::IntProperty("Load to (s ago)", 0,
      "Age of the newest detection to load [s]. 0 loads detections of any age, even ones stamped in the future (the "
      "clock of a replayed recording is usually behind the stored detections). Set a positive value to load only "
      "detections older than this.", historyCategory);
  historyToProperty_->setMin(0);
  historyToProperty_->setMax(365 * 24 * 3600);
  historyClassesProperty_ = new rviz_common::properties::StringProperty("Classes", "",
      "Comma-separated list of the classes to load. Leave empty to load all of them.",
      historyCategory);
  historyCamerasProperty_ = new rviz_common::properties::StringProperty("Cameras", "",
      "Comma-separated list of the camera frames to load the detections of. The detections of several cameras can be "
      "stored in one storage and their frames are the only thing to tell them apart. Leave empty to load all of "
      "them.", historyCategory);
  historyZoneProperty_ = new rviz_common::properties::EnumProperty("Keep-out zones",
      "All detections",
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
  historyDelayProperty_ = new rviz_common::properties::FloatProperty("Start delay", 2.,
      "How long to wait with loading the stored detections after RViz starts (or after the settings change), in "
      "seconds. Loading a lot of old detections would otherwise slow down the start of RViz.",
      historyCategory);
  historyDelayProperty_->setMin(0.);
  historyDelayProperty_->setMax(600.);
  historyBatchProperty_ = new rviz_common::properties::IntProperty("Batch size", 100,
      "How many stored detections are added at once. The batches are added with pauses between them, so that the "
      "interface stays responsive while the history is loading.", historyCategory);
  historyBatchProperty_->setMin(1);
  historyBatchProperty_->setMax(10000);
  historyPauseProperty_ = new rviz_common::properties::FloatProperty("Batch pause", .2,
      "How long to wait between the batches of the stored detections [s].", historyCategory);
  historyPauseProperty_->setMin(0.);
  historyPauseProperty_->setMax(60.);

  // A property sends its signals to its parent, which is the category, not the display. The display therefore has to
  // connect to the properties of the categories explicitly.
  for (auto* category : {markerCategory, mergeCategory, labelCategory, imageCategory,
         selectionCategory,
         historyCategory})
  {
    for (int i = 0; i < category->numChildren(); ++i) {
      auto* property = category->childAt(i);
      connect(property, SIGNAL(changed()), this, SLOT(redraw()));
    }
  }
  connect(imageToleranceProperty_, SIGNAL(changed()), this, SLOT(redraw()));
  connect(statusDebugProperty_, SIGNAL(changed()), this, SLOT(updateStatusDebug()));
  connect(selectionTopicProperty_, SIGNAL(changed()), this, SLOT(updateSelectionTopic()));
  connect(selectionTopicNameProperty_, SIGNAL(changed()), this, SLOT(updateSelectionTopic()));
  // Anything in the query changes what is in the storage, so the history has to be loaded again.
  const std::vector<rviz_common::properties::Property*> historyProperties {
    historyEnabledProperty_, storageDirProperty_, globalFrameProperty_, historyFromProperty_,
    historyToProperty_,
    historyClassesProperty_, historyCamerasProperty_, historyZoneProperty_};
  for (auto* property : historyProperties) {
    connect(property, SIGNAL(changed()), this, SLOT(reloadHistory()));
  }
}


Detection2DArrayDisplay::~Detection2DArrayDisplay() {
  if (imageSubscriber_) {
    imageSubscriber_.shutdown();
  }
  clearDrawn();
  if (previewObject_ && scene_manager_) {
    scene_manager_->destroyManualObject(previewObject_);
  }
  if (annotationObject_ && scene_manager_) {
    scene_manager_->destroyManualObject(annotationObject_);
  }
  textures_.reset();
  images_.reset();
}

void Detection2DArrayDisplay::onInitialize() {
  RTDClass::onInitialize();

  prefix_ = resourcePrefix();
  images_ = std::make_unique<ImageCache>(MAX_CACHED_IMAGES);
  // The previews which were shown recently are kept in memory, so that a detection whose preview was pushed out of
  // the view by a more important one can show its preview again without taking it from the camera image once more.
  textures_ = std::make_unique<TextureCache>(std::max(size_t{8}, 2 * static_cast<size_t>(
        maxPreviewsProperty_->getInt())), prefix_);

  overlayNode_ = scene_node_->createChildSceneNode();
  previewObject_ = scene_manager_->createManualObject(prefix_ + "/previews");
  previewObject_->setQueryFlags(0);  // the annotations are not pickable
  previewObject_->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);
  overlayNode_->attachObject(previewObject_);
  // The frames of the previews and the lines to their detections are drawn over everything, including the previews.
  annotationObject_ = scene_manager_->createManualObject(prefix_ + "/annotations");
  annotationObject_->setQueryFlags(0);
  annotationObject_->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY + 1);
  overlayNode_->attachObject(annotationObject_);

  // A topic property which does not know the node crashes RViz when the user opens its list of topics to choose one.
  imageTopicProperty_->initialize(rviz_ros_node_);
  zonesTopicProperty_->initialize(rviz_ros_node_);

  auto node = rviz_ros_node_.lock();
  if (node) {
    imageTransport_ = std::make_unique<cras::ImageTransport>(
        cras::ImageTransport::RequiredInterfaces(*node->get_raw_node()));
    updateImageTopic();
    updateSelectionTopic();
    updateZonesTopic();
  }
  updateStatus();
}

std::string Detection2DArrayDisplay::resourcePrefix() const{
  // The rendering resources (textures, materials, entities) live in the whole process, but several RViz instances can
  // run in it (e.g. several rqt_rviz widgets), so the names have to say which instance and display they belong to.
  static std::atomic<unsigned> counter {0};
  std::string node = "rviz";
  const auto rosNode = rviz_ros_node_.lock();
  if (rosNode) {
    node = rosNode->get_node_name();
  }
  return "cras_rviz_vision_msgs_" + sanitize(node) + '_' + std::to_string(++counter) + '_' +
  sanitize(getName().toStdString());
}

const std::string& Detection2DArrayDisplay::markerMaterial(
    const Ogre::ColourValue& colour,
    bool highlighted) {
  // The detections share the materials of their markers - the colour of a detection is the only thing which differs
  // between them, and creating a material for every one of thousands of detections would be needlessly expensive.
  // The highlighted detection has a material of its own because it is drawn over everything else.
  const auto key = ((static_cast<int>(std::lround(7. * colour.r)) << 9) |
                    (static_cast<int>(std::lround(7. * colour.g)) << 6) |
                    (static_cast<int>(std::lround(7. * colour.b)) << 3) |
                    static_cast<int>(std::lround(7. * colour.a))) << 1 |
                   (highlighted ? 1 : 0);
  const auto known = markerMaterials_.find(key);
  if (known != markerMaterials_.end()) {
    return known->second;
  }
  const auto name = prefix_ + "/marker_" + std::to_string(key);
  auto material = rviz_rendering::MaterialManager::createMaterialWithLighting(name);
  // The same shading rviz_rendering::Shape uses for the sphere of a detected point.
  material->getTechnique(0)->setAmbient(colour * .5F);
  material->getTechnique(0)->setDiffuse(colour);
  if (highlighted) {
    // Nothing is allowed to hide the detection the user asked for, not even the markers of the other detections.
    material->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    material->setDepthCheckEnabled(false);
    material->setDepthWriteEnabled(false);
  }
  return markerMaterials_.insert({key, name}).first->second;
}

int64_t Detection2DArrayDisplay::nowNs(bool* rosClock) const{
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

void Detection2DArrayDisplay::resolvePendingDetections() {
  const auto globalFrame = globalFrameProperty_->getStdString();
  if (globalFrame.empty()) {
    return;  // the detections are remembered in the frame they arrived in, there is nothing to resolve
  }
  const auto now = nowNs();
  if (now - lastResolveNs_ < static_cast<int64_t>(RETRY_TRANSFORMS_PERIOD * 1e9)) {
    return;
  }
  lastResolveNs_ = now;
  bool resolved = false;
  const auto transformer = context_->getFrameManager()->getTransformer();
  if (!transformer) {
    return;
  }
  std::lock_guard<std::mutex> lock(detectionsMutex_);
  // The detections which arrived together share the frame and the time, so the transform is looked up only once for
  // them (a dense scene would otherwise mean thousands of lookups until the transform finally appears).
  std::map<std::pair<std::string, int64_t>, std::optional<tf2::Transform>> lookups;
  for (auto& detection : detections_.detections()) {
    if (!detection.hasPose || detection.inGlobalFrame) {
      continue;
    }
    if (now - detection.resolveTryNs < static_cast<int64_t>(RETRY_TRANSFORMS_PERIOD * 1e9)) {
      continue;
    }
    detection.resolveTryNs = now;
    // The detections from the storage are old, and their positions in the global frame are meant to be valid for any
    // time, so the newest transform is the right one to take for them. It also means the detections of one frame share
    // a single lookup, which matters for the thousands of stored detections.
    const auto stamp = detection.history ? int64_t{0} : detection.stamp.nanoseconds();
    const auto key = std::make_pair(detection.frameId, stamp);
    auto lookup = lookups.find(key);
    if (lookup == lookups.end()) {
      lookup = lookups.emplace(key, lookupTransform(transformer, globalFrame, detection.frameId,
        rclcpp::Time(stamp, RCL_ROS_TIME))).first;
    }
    if (!lookup->second) {
      continue;  // the transform did not come yet; the number of such detections is reported in the status
    }
    applyTransform(*lookup->second, detection.pose);
    detection.frameId = globalFrame;
    detection.inGlobalFrame = true;
    resolved = true;
    layoutDirty_ = true;
  }
  if (resolved) {
    // The identification of a detection includes the frame its pose is in, so the detections which just moved have to
    // be indexed again. Without it, the next message with the same ID would not find them and the object would be
    // remembered (and drawn) twice.
    detections_.reindex();
  }
}

void Detection2DArrayDisplay::updateImageTopic() {
  const auto topic = imageTopicProperty_->getTopicStd();
  const auto compressed = ImageTopicProperty::compressed(topic,
      imageTopicProperty_->selectedMessageType());
  const bool subscribed =
    compressed ? static_cast<bool>(compressedSubscriber_) : static_cast<bool>(imageSubscriber_);
  if (topic == imageTopicName_ && compressed == imageTopicCompressed_ &&
      (topic.empty() || subscribed))
  {
    // RViz re-emits the topic property changed signal whenever it refreshes the list of the available topics, so do
    // not disturb a running subscription (and the images it brought) unless the topic really changed.
    return;
  }
  imageTopicName_ = topic;
  imageTopicCompressed_ = compressed;
  if (imageSubscriber_) {
    imageSubscriber_.shutdown();
    imageSubscriber_ = image_transport::Subscriber();
  }
  compressedSubscriber_.reset();
  if (images_) {
    images_->clear();
  }
  if (!topic.empty()) {
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
  }
  layoutDirty_ = true;
  updateStatus();
  queueRender();
}

void Detection2DArrayDisplay::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
  images_->add(msg, MAX_IMAGE_SIZE);
}

void Detection2DArrayDisplay::compressedImageCallback(
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr& msg) {
  images_->add(msg, MAX_IMAGE_SIZE);
}

void Detection2DArrayDisplay::zonesCallback(
    const polygon_msgs::msg::Polygon2DCollection::ConstSharedPtr& msg) {
  std::vector<KeepoutZones::Zone> zones;
  zones.reserve(msg->polygons.size());
  for (const auto& polygon : msg->polygons) {
    KeepoutZones::Zone zone;
    zone.reserve(polygon.points.size());
    for (const auto& point : polygon.points) {
      zone.emplace_back(point.x, point.y);
    }
    zones.push_back(std::move(zone));
  }
  bool changed = false;
  {
    std::lock_guard<std::mutex> lock(detectionsMutex_);
    changed = zones_.set(msg->header.frame_id, std::move(zones));
    if (changed) {
      // The zones can be drawn, moved or deleted while the display runs, and the detections are remembered for a long
      // time, so what the zones say about them is not known anymore and has to be told again.
      for (auto& detection : detections_.detections()) {
        detection.inZoneNow.reset();
        detection.nowZoneIds.clear();
        detection.zoneTryNs = 0;
      }
    }
  }
  if (!changed) {
    return;  // the zones are published repeatedly and the publisher usually sends the same ones again and again, and
             // asking again for what has not changed would only make the detections and their previews flicker
  }
  // What the zones say about the remembered detections is told again by the next drawn frame (together with the status,
  // which would otherwise shortly say that no detection is in a zone just because they were all forgotten now).
  layoutDirty_ = true;
  queueRender();
}

bool Detection2DArrayDisplay::markZones() {
  inZoneCount_ = 0;
  unknownZoneCount_ = 0;
  bool changed = false;
  // The detections which arrived together share the frame and the time, so the transform to the frame of the zones is
  // looked up only once for them (a dense scene would otherwise mean thousands of lookups in a single frame).
  std::map<std::pair<std::string, int64_t>, std::optional<tf2::Transform>> lookups;
  const auto transformer = context_->getFrameManager()->getTransformer();
  const auto inZones = [this, &transformer, &lookups](const StoredDetection& detection, int64_t stamp) {
                         // The place of a detection which is already in the frame of the zones needs no transform, and
                         // asking for one would be wrong - the frame may be the frame of the view, which is never in
                         // the transform buffer as a frame of its own.
                         const auto& frame = detection.frameId.empty() ? frameId : detection.frameId;
                         if (frame.empty() || frame == zones_.getFrame()) {
                           return std::optional<std::pair<double, double>>(std::make_pair(
                             detection.pose.position.x, detection.pose.position.y));
                         }
                         const auto key = std::make_pair(frame, stamp);
                         auto lookup = lookups.find(key);
                         if (lookup == lookups.end()) {
                           lookup = lookups.emplace(key,
                             lookupTransform(transformer, zones_.getFrame(), frame,
                             rclcpp::Time(stamp, RCL_ROS_TIME))).first;
                         }
                         if (!lookup->second) {
                           return std::optional<std::pair<double, double>>();
                         }
                         const auto where = *lookup->second * tf2::Vector3(detection.pose.position.x,
                           detection.pose.position.y, detection.pose.position.z);
                         return std::optional<std::pair<double, double>>(
                           std::make_pair(where.x(), where.y()));
                       };
  const auto now = nowNs();
  for (auto& detection : detections_.detections()) {
    // What the zones say about a detection is known until the zones themselves change (which asks for the membership
    // again) or until the detection is replaced by a newer report of the object.
    if (!detection.inZoneNow && !zones_.empty() && detection.hasPose &&
        now - detection.zoneTryNs >= static_cast<int64_t>(RETRY_TRANSFORMS_PERIOD * 1e9))
    {
      detection.zoneTryNs = now;
      // The detections from the storage are old and their positions are meant to be valid at any time, so the newest
      // transform is the right one to take for them right away. For the others it is only a fallback for the case that
      // their own time is not covered by the transform buffer - the zones do not move, so the newest transform still
      // tells where the place of the detection is in them.
      auto where = inZones(detection,
          detection.history ? int64_t{0} : detection.stamp.nanoseconds());
      if (!where && detection.stamp.nanoseconds() != 0) {
        where = inZones(detection, 0);
      }
      if (where) {
        detection.nowZoneIds = zones_.zonesAt(where->first, where->second);
        detection.inZoneNow = !detection.nowZoneIds.empty();
        changed = true;
      }
    }
    inZoneCount_ += insideZone(detection) ? 1U : 0U;
    if (!detection.inZoneNow && !detection.hasZone) {
      ++unknownZoneCount_;  // neither the zones know it, nor was anything recorded with the detection
    }
  }
  zoneCount_ = zones_.size();
  return changed;
}

void Detection2DArrayDisplay::processMessage(vision_msgs::msg::Detection2DArray::ConstSharedPtr msg) {
  // The rate of the incoming messages, measured over a short while back so that it reacts to a stopped detector. RViz
  // only knows the average since the subscription was created, which stays almost zero for a while.
  const auto arrived = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  if (lastMessageNs_ > 0 && arrived > lastMessageNs_) {
    const auto instant = 1e9 / static_cast<double>(arrived - lastMessageNs_);
    messageRate_ = messageRate_ > 0. ? .9 * messageRate_ + .1 * instant : instant;
  }
  lastMessageNs_ = arrived;
  ++messagesReceived_;

  const auto globalFrame = globalFrameProperty_->getStdString();
  const rclcpp::Time stamp(msg->header.stamp, RCL_ROS_TIME);

  // The detections are remembered in the global frame whenever it is available, because the transform from it to the
  // frame of the view is static and therefore stays valid for an arbitrarily old detection. The transform of the whole
  // message is looked up once (and not once per detection), which matters for messages with hundreds of detections.
  // It has to be the transform of the message time: the frame of a detector moves, so the newest transform would place
  // the detections somewhere they were not (and they would keep moving as the sensor moves). When the transform is not
  // available (yet), the detections stay in their frame and resolvePendingDetections() retries later.
  const auto toGlobal = lookupTransform(context_->getFrameManager()->getTransformer(), globalFrame,
      msg->header.frame_id, stamp);

  std::vector<StoredDetection> detections;
  detections.reserve(msg->detections.size());
  for (const auto& detection : msg->detections) {
    StoredDetection stored;
    stored.stamp = rclcpp::Time(msg->header.stamp, RCL_ROS_TIME);
    stored.id = detection.id;
    stored.frameId = msg->header.frame_id;
    // The frame the detection came in stays remembered even after its pose is moved to the global frame, so that the
    // next message from the same camera finds the detection again instead of adding a second copy of it.
    stored.sourceFrameId = msg->header.frame_id;
    // The frame of the message is the frame of the camera the detection was seen by. It has to be remembered
    // separately, because the pose below is moved to the global frame and several cameras can share the topic.
    stored.cameraFrame = msg->header.frame_id;

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
      stored.hasPose = std::isfinite(stored.pose.position.x) &&
                       std::isfinite(stored.pose.position.y) &&
                       std::isfinite(stored.pose.position.z);
      if (stored.hasPose && toGlobal) {
        applyTransform(*toGlobal, stored.pose);
        stored.frameId = globalFrame;
        stored.inGlobalFrame = true;
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
    if (!accumulateProperty_->getBool()) {
      detections_.clearLive();  // the base class clears everything, including the detections from the storage
    }
    if (const auto topic = topic_property_->getTopicStd(); topic != detectionsTopicName_) {
      // RViz resets the display whenever anything in its properties changes; the accumulated detections may only be
      // dropped when we start receiving another topic.
      detectionsTopicName_ = topic;
      detections_.clear();
      history_.loaded = 0;
    }
    // The detections of the previous messages are kept, so that an empty message does not hide them and so that more
    // detections can be shown than a single message contains.
    detections_.add(std::move(detections), accumulateProperty_->getBool());
    frameId = msg->header.frame_id;
  }
  layoutDirty_ = true;
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
      // The same colours the alarms of the keep-out zone monitor stand for: red for a detection in a zone. What the
      // zones say now wins over what was recorded with the detection - the zones can change after a detection was
      // remembered, and a historical detection is only red while it really stands in a zone.
      if (!detection.inZoneNow && !detection.hasZone) {
        return Ogre::ColourValue{.6F, .6F, .6F, 1.F};
      }
      return (detection.inZoneNow ? *detection.inZoneNow : detection.inZone) ?
        Ogre::ColourValue{1.F, .15F, .1F, 1.F} : Ogre::ColourValue{.2F, .8F, .3F, 1.F};
    case ColorMode::NONE: return Ogre::ColourValue{.3F, .7F, 1.F, 1.F};
    case ColorMode::PROBABILITY:
    default:
    {
      const auto score = static_cast<float>(std::max(0., std::min(1., detection.probability)));
      return Ogre::ColourValue(1.F - score, score, .1F, 1.F);
    }
  }
}

void Detection2DArrayDisplay::update(float wall_dt, float ros_dt) {
  Display::update(wall_dt, ros_dt);
  if (!images_ || !textures_) {
    return;  // not initialized yet
  }

  // The previews which are not shown anymore are destroyed here, because the rendering resources may only be
  // destroyed in the rendering thread.
  textures_->setLimit(std::max(size_t{8},
      2 * static_cast<size_t>(std::max(1, maxPreviewsProperty_->getInt()))));
  textures_->enforceLimits();

  bool zoneChanged = false;
  {
    std::lock_guard<std::mutex> lock(detectionsMutex_);
    detections_.setLimit(static_cast<size_t>(std::max(1, maxMarkersProperty_->getInt())));
    detections_.setDeduplication({merge3dProperty_->getFloat(), merge2dProperty_->getFloat(),
          mergeTimeProperty_->getFloat()});
    // Which of the detections are inside a keep-out zone has to be told before their age is looked at, because a
    // detection in a zone can be allowed to stay for a different time than one which is out of all of them.
    zoneChanged = markZones();
    // The detections which are not reported again do not stay forever (unless the user asked for it), and the limit on
    // the number of displayed markers is kept here rather than in the drawing, so that the remembered detections do
    // not grow without a bound.
    if (detections_.prune(nowNs(), static_cast<double>(lifetimeProperty_->getFloat()),
        static_cast<double>(inZoneLifetimeProperty_->getFloat())) > 0)
    {
      layoutDirty_ = true;
    }
  }
  if (zoneChanged) {
    layoutDirty_ = true;
  }

  updateHistory(wall_dt);
  updateHover();
  updateSelection();
  resolvePendingDetections();

  // The detections have to be projected by the camera of the currently active view. The viewport it renders to tells
  // us how large the view is, which the placement of the annotations depends on.
  auto* viewManager = context_->getViewManager();
  auto* viewController = viewManager ? viewManager->getCurrent() : nullptr;
  auto* camera = viewController ? viewController->getCamera() : nullptr;
  if (!camera) {
    report("Detections", rviz_common::properties::StatusProperty::Level::Warn,
      "The display needs an active view to draw the detections to.");
    return;
  }
  auto* renderPanel = viewManager->getRenderPanel();
  auto* renderWindow = renderPanel ? renderPanel->getRenderWindow() : nullptr;
  auto* viewport =
    renderWindow ? rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(renderWindow) : nullptr;
  if (!viewport) {
    viewport = camera->getViewport();
  }
  if (!viewport || viewport->getActualWidth() < 2. || viewport->getActualHeight() < 2.) {
    return;  // the view is not rendered yet
  }
  if (!mouseWatcher_) {
    // The rendering window is a QWindow embedded in the RViz widget, so listening on it is the only way a display can
    // learn where the mouse is (the mouse events all belong to the active tool).
    mouseWatcher_ = std::make_unique<MouseWatcher>(*this);
    QWindow* window = renderWindow;
    window->installEventFilter(mouseWatcher_.get());
  }
  // The mouse position is in the window coordinates, while the detections are projected to the viewport pixels.
  mouseScaleX_ = viewport->getActualWidth() / std::max(1.,
      static_cast<double>(renderWindow->width()));
  mouseScaleY_ = viewport->getActualHeight() / std::max(1.,
      static_cast<double>(renderWindow->height()));

  // Drawing everything is the most expensive thing the display does, so it only happens when something actually
  // changed. An idle frame is a handful of comparisons.
  const auto viewKey = [&camera, &viewport]() {
                         const auto& position = camera->getDerivedPosition();
                         const auto& orientation = camera->getDerivedOrientation();
                         std::ostringstream key;
                         key << std::setprecision(6) << position.x << position.y << position.z << orientation.x <<
                           orientation.y << orientation.z << orientation.w << camera->getOrthoWindowHeight() <<
                           camera->getFOVy() << viewport->getActualWidth() <<
                           viewport->getActualHeight();
                         return key.str();
                       }();
  if (viewKey != lastViewKey_) {
    layoutDirty_ = true;
  }
  refreshAge_ += wall_dt;
  if (refreshAge_ >= transformRefresh_) {
    // The transforms change without any other notice, and a transform which was not available yet can appear any time
    // (the static transforms of a system are typically published only some time after it starts).
    layoutDirty_ = true;
  }
  const auto revision = detections_.revision();
  if (revision != layoutRevision_) {
    layoutDirty_ = true;
  }

  if (!layoutDirty_) {
    updateStatus();
    return;
  }
  // The detections arrive much faster than a person can follow, and rearranging the previews with every message makes
  // the view flicker, so the layout only catches up once per the configured period. What the detections are and what
  // the display says about them is reported right away, only the picture waits.
  layoutAge_ += wall_dt;
  if (!layoutUrgent_ && layoutAge_ < static_cast<double>(updatePeriodProperty_->getFloat())) {
    updateStatus();
    return;
  }
  layoutUrgent_ = false;
  layoutAge_ = 0.;
  lastViewKey_ = viewKey;
  layoutRevision_ = revision;
  layoutDirty_ = false;
  layout(*camera, *viewport);
  updateStatus();
}

void Detection2DArrayDisplay::updateStatusDebug() {
  statusDebug_ = statusDebugProperty_->getBool();
  lastStatusLogNs_.clear();
  updateStatus();
}

void Detection2DArrayDisplay::onEnable() {
  RTDClass::onEnable();
  // Everything was destroyed when the display was disabled and the transforms may have changed in the meantime, so the
  // next frame has to lay the detections out again.
  refreshAge_ = transformRefresh_;
  layoutDirty_ = true;
  queueRender();
}

void Detection2DArrayDisplay::onDisable() {
  // The rendering resources of the display must not stay alive after the scene they belong to was hidden: the preview
  // textures and materials live in the whole RViz process and re-enabling the display would find them pointing to
  // objects which are not there anymore.
  clearDrawn();
  clearLabels();
  shownPreviews_.clear();
  knownSerials_.clear();
  hoveredSerial_ = 0;
  hoverCandidate_ = 0;
  clearSelection();
  RTDClass::onDisable();
}

void Detection2DArrayDisplay::reset() {
  RTDClass::reset();
  clearDrawn();
  if (textures_) {
    textures_->clear();
  }
  knownSerials_.clear();
  shownPreviews_.clear();
  hoveredSerial_ = 0;
  hoverCandidate_ = 0;
  clearSelection();
  layoutDirty_ = true;
  updateStatus();
}

void Detection2DArrayDisplay::clearDrawn() {
  for (auto& marker : markers_) {
    if (marker.second->entity && scene_manager_) {
      scene_manager_->destroyEntity(marker.second->entity);
    }
    if (marker.second->node && scene_manager_) {
      scene_manager_->destroySceneNode(marker.second->node);
    }
  }
  markers_.clear();
  for (auto& label : labels_) {
    if (label->text && label->node) {
      label->node->detachObject(label->text.get());
    }
    label->text.reset();  // the text is owned by the label, the node only draws it
    if (label->node && scene_manager_) {
      scene_manager_->destroySceneNode(label->node);
    }
    label->node = nullptr;
    label->serial = 0;
    label->caption.clear();
  }
  labelOf_.clear();
  if (previewObject_) {
    previewObject_->clear();
  }
  if (annotationObject_) {
    annotationObject_->clear();
  }
  items_.clear();
  previewCount_ = 0;
  labelCount_ = 0;
  visualizedCount_ = 0;
}

void Detection2DArrayDisplay::clearLabels() {
  // The text objects are tied to the scene nodes they hang on and both belong to the scene of the display, so they are
  // destroyed together here (which onDisable() has to do before the scene is hidden).
  for (auto& label : labels_) {
    label->text.reset();
    if (label->node && scene_manager_) {
      scene_manager_->destroySceneNode(label->node);
    }
    label->node = nullptr;
    label->serial = 0;
    label->caption.clear();
  }
  labels_.clear();
  labelOf_.clear();
}

const std::string& Detection2DArrayDisplay::annotationMaterial() {
  if (annotationMaterial_.empty()) {
    // The frames and the lines are drawn with the colour of their vertices, so one material is enough for all of them
    // and the hovered preview can be highlighted without creating another one.
    annotationMaterial_ = prefix_ + "/annotations";
    auto& manager = Ogre::MaterialManager::getSingleton();
    if (!manager.resourceExists(annotationMaterial_)) {
      const auto material = manager.create(annotationMaterial_,
              Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
      material->setLightingEnabled(false);
      material->setDepthCheckEnabled(false);
      material->setDepthWriteEnabled(false);
      material->setCullingMode(Ogre::CULL_NONE);
      auto* pass = material->getTechnique(0)->getPass(0);
      pass->setVertexColourTracking(Ogre::TVC_DIFFUSE);
      pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    }
  }
  return annotationMaterial_;
}

const std::set<std::string>& Detection2DArrayDisplay::cameraFrames() {
  std::set<std::string> frames;
  if (images_) {
    for (const auto& source : images_->sources()) {
      if (source.images > 0) {
        frames.insert(source.frameId);
      }
    }
  }
  cameraFrames_.swap(frames);
  return cameraFrames_;
}

std::string Detection2DArrayDisplay::wantedFrame(const std::string& cameraFrame) {
  const auto configured = cameraFrameProperty_->getStdString();
  if (configured == "*") {
    return {};  // the user asked for the closest image of any camera
  }
  if (!configured.empty()) {
    return configured;
  }
  if (cameraFrame.empty()) {
    return {};
  }
  // The images are looked for in the camera the detection was reported by, unless the detections do not come in camera
  // frames at all (a detector often reports them in a frame of the robot) - then the time is all there is to match by.
  return cameraFrames().count(cameraFrame) ? cameraFrame : std::string();
}

uint64_t Detection2DArrayDisplay::nearestMarker(double x, double y, double* nearestGap) const{
  // The markers are looked for in the projections of the previous layout, so no new projection is needed and looking
  // for the closest one costs nothing but a few distance comparisons.
  const auto allowed = std::max(HOVER_RADIUS, MIN_HIT_RADIUS);
  auto best = uint64_t{0};
  auto bestGap = std::numeric_limits<double>::max();
  auto nearest = std::numeric_limits<double>::max();
  for (const auto& item : items_) {
    const auto dx = item.screen.x - x;
    const auto dy = item.screen.y - y;
    // How far the place is from the drawn sphere; negative when it is inside it.
    const auto gap = std::sqrt(dx * dx + dy * dy) - item.radius;
    nearest = std::min(nearest, gap);
    if (gap <= allowed && gap < bestGap) {
      bestGap = gap;
      best = item.serial;
    }
  }
  if (nearestGap) {
    *nearestGap = nearest;
  }
  return best;
}

void Detection2DArrayDisplay::updateHover() {
  const auto candidate = mouseInside_ ? nearestMarker(mouseX_, mouseY_) : uint64_t{0};
  // The delay is measured by the wall clock, not by the time between the rendered frames: the display only renders
  // when something changes, so a hover which started at an idle moment would never be recognised.
  const auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  if (candidate != hoverCandidate_) {
    hoverCandidate_ = candidate;
    hoverSinceNs_ = now;
  }
  const auto delay = static_cast<double>(hoverDelayProperty_->getFloat());
  const bool waited = now - hoverSinceNs_ >= static_cast<int64_t>(delay * 1e9);
  const auto wanted = waited ? hoverCandidate_ : uint64_t{0};
  if (wanted != hoveredSerial_) {
    hoveredSerial_ = wanted;
    layoutDirty_ = true;
    layoutUrgent_ = true;  // the person pointed at the detection, so its preview should come up right away
  }
  if (hoverCandidate_ != 0 && !waited) {
    queueRender();  // keep the frames coming, so that the delay can elapse
  }
  if (layoutDirty_) {
    queueRender();
  }
}

void Detection2DArrayDisplay::updateSelectionTopic() {
  pointSubscriber_.reset();
  headerSubscriber_.reset();
  selectionTopicName_.clear();
  const auto topic = selectionTopicNameProperty_->getStdString();
  const auto mode = static_cast<SelectionMode>(selectionTopicProperty_->getOptionInt());
  if (topic.empty() || mode == SelectionMode::NONE) {
    layoutDirty_ = true;
    updateStatus();
    return;
  }
  const auto node = rviz_ros_node_.lock();
  if (!node) {
    return;
  }
  // There is no way to tell which of the two message types a topic carries before it is subscribed to, and subscribing
  // to it with the wrong one would silently deliver nothing, so the user says which one it is (see the property).
  selectionTopicName_ = topic;
  try {
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();
    if (mode == SelectionMode::POINT) {
      pointSubscriber_ =
        node->get_raw_node()->create_subscription<geometry_msgs::msg::PointStamped>(topic, qos,
            [this](const geometry_msgs::msg::PointStamped::ConstSharedPtr& msg) {
              selectedPointCallback(msg);
          });
    } else {
      headerSubscriber_ = node->get_raw_node()->create_subscription<std_msgs::msg::Header>(topic,
          qos,
          [this](const std_msgs::msg::Header::ConstSharedPtr& msg) {
            selectedHeaderCallback(msg);
          });
    }
  } catch (const std::exception& e) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("cras_rviz_vision_msgs.Detection2DArrayDisplay"),
      "Failed to subscribe to the topic of clicked images \"" << topic << "\": " << e.what());
    selectionTopicName_.clear();
  }
  layoutDirty_ = true;
  updateStatus();
  if (selectionTopicName_.empty()) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("cras_rviz_vision_msgs.Detection2DArrayDisplay"),
      "Not listening for clicked images (topic \"" << topic << "\", type " <<
          selectionTopicProperty_->getString().toStdString() << ").");
  } else {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("cras_rviz_vision_msgs.Detection2DArrayDisplay"),
      "Listening for clicked images on " << selectionTopicName_ << " (" <<
          selectionTopicProperty_->getString().toStdString() << " messages).");
  }
}

void Detection2DArrayDisplay::updateZonesTopic() {
  const auto topic = zonesTopicProperty_->getTopicStd();
  if (topic == zonesTopicName_ && (topic.empty() || zonesSub_)) {
    return;  // the property is emitted again whenever RViz refreshes the list of topics
  }
  zonesTopicName_ = topic;
  zonesSub_.reset();
  {
    std::lock_guard<std::mutex> lock(detectionsMutex_);
    zones_.clear();
    for (auto& detection : detections_.detections()) {
      detection.inZoneNow.reset();
      detection.nowZoneIds.clear();
      detection.zoneTryNs = 0;
    }
  }
  if (!topic.empty()) {
    try {
      const auto node = rviz_ros_node_.lock();
      if (node) {
        // The zones are drawn once and published repeatedly (or latched), so a display which starts later has to get
        // them too - that is why the latched quality of service is asked for (as the zone tool publishes them).
        zonesSub_ = node->get_raw_node()->create_subscription<polygon_msgs::msg::Polygon2DCollection>(topic,
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
            [this](const polygon_msgs::msg::Polygon2DCollection::ConstSharedPtr& msg) {
              zonesCallback(msg);
            });
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("cras_rviz_vision_msgs.Detection2DArrayDisplay"),
        "Failed to subscribe to the topic of keep-out zones \"" << topic << "\": " << e.what());
      zonesTopicName_.clear();
    }
  }
  layoutDirty_ = true;
  updateStatus();
}

void Detection2DArrayDisplay::selectedPointCallback(
    const geometry_msgs::msg::PointStamped::ConstSharedPtr& msg) {
  Selection selection;
  selection.cameraFrame = msg->header.frame_id;
  selection.stamp = msg->header.stamp;
  selection.tolerance = static_cast<double>(selectionToleranceProperty_->getFloat());
  selection.hasPoint = true;
  selection.x = msg->point.x;
  selection.y = msg->point.y;
  select(selection);
}

void Detection2DArrayDisplay::selectedHeaderCallback(
    const std_msgs::msg::Header::ConstSharedPtr& msg) {
  Selection selection;
  selection.cameraFrame = msg->frame_id;
  selection.stamp = msg->stamp;
  selection.tolerance = static_cast<double>(selectionToleranceProperty_->getFloat());
  select(selection);
}

void Detection2DArrayDisplay::select(const Selection& selection) {
  std::vector<uint64_t> found;
  std::string cameraFrame;
  {
    std::lock_guard<std::mutex> lock(detectionsMutex_);
    found = detections_.select(selection);
    if (!found.empty()) {
      // The user clicked in a camera image, and that is the image the preview should show - not the one the detection
      // was first seen in, which can be a long time ago and cannot be shown anyway when it is gone from the cache.
      detections_.retakePreview(found.front(), selection);
      for (const auto& detection : detections_.detections()) {
        if (detection.serial == found.front()) {
          cameraFrame = detection.cameraFrame;
          break;
        }
      }
    }
  }
  selectionMissed_ = found.empty();
  if (!found.empty()) {
    pinClickedImage(found.front(), selection.stamp, cameraFrame);
    select(found.front());
  } else {
    updateStatus();
  }
}

void Detection2DArrayDisplay::pinClickedImage(uint64_t serial, const rclcpp::Time& stamp,
    const std::string& cameraFrame) {
  // The picture the highlighted detection should show is the image the user clicked in, so take hold of it before it
  // leaves the cache: the click can come from an image older than what the cache keeps (the visualization strip only
  // refreshes once in a while), and the layout which is going to show the preview runs up to a second later.
  // Which camera to look in is what the user configured, or the camera the detection was reported in; unlike
  // \ref wantedFrame it is not checked against the cameras which really deliver images - if nothing is found for it,
  // the closest image of any camera is still a better answer than no picture at all.
  const auto configured = cameraFrameProperty_->getStdString();
  const std::string frame = configured == "*" ? std::string() : (configured.empty() ? cameraFrame : configured);
  auto image = images_->get(stamp, static_cast<double>(imageToleranceProperty_->getFloat()), frame);
  if (!image) {
    // The exact image is gone, but the moment is not - the closest one of that camera still shows what the user saw.
    image = images_->nearest(stamp, frame);
  }
  if (!image && !frame.empty()) {
    image = images_->nearest(stamp);
  }
  std::lock_guard<std::mutex> lock(detectionsMutex_);
  pinned_.serial = image ? serial : 0;
  pinned_.image = image;
}

void Detection2DArrayDisplay::select(uint64_t serial) {
  if (serial == 0) {
    return;
  }
  const auto duration = static_cast<double>(highlightProperty_->getFloat());
  // Measured by the wall clock: the highlight is a piece of the interface, and it should not freeze when the simulated
  // clock stops, nor run at a different speed than the user expects.
  const auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  // Asking for the same detection again starts the time from the beginning; the detections which are not shown
  // anymore are dropped by updateSelection().
  highlightedUntilNs_[serial] = now + static_cast<int64_t>(std::max(0., duration) * 1e9);
  if (std::find(requested_.begin(), requested_.end(), serial) == requested_.end()) {
    requested_.push_back(serial);
    // Nobody clicks on dozens of detections at once, but do not let the list grow if they do.
    if (requested_.size() > MAX_REQUESTED_DETECTIONS) {
      requested_.erase(requested_.begin());
    }
  }
  selectionMissed_ = false;
  layoutDirty_ = true;
  // The detection the user asked for goes to the front of the queue - waiting for the next layout period for its
  // preview to appear would feel like the click was ignored.
  layoutUrgent_ = true;
  queueRender();
  if (serial != selectedSerial_) {
    // A click is a rare event made by a person who wants to see something, so it is worth saying what it led to.
    RCLCPP_INFO_STREAM(rclcpp::get_logger("cras_rviz_vision_msgs.Detection2DArrayDisplay"),
      "Highlighting detection " << serial << " for " << duration << " s.");
  }
}

void Detection2DArrayDisplay::updateSelection() {
  const auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  for (auto highlight = highlightedUntilNs_.begin(); highlight != highlightedUntilNs_.end();) {
    if (highlight->second <= now) {
      highlight = highlightedUntilNs_.erase(highlight);
    } else {
      ++highlight;
    }
  }
  // The last detection the user clicked on is the one to show; when it is dropped from the displayed detections, the
  // one clicked before it (if it is still shown and asked for) takes its place.
  for (auto requested = requested_.begin(); requested != requested_.end();) {
    if (highlightedUntilNs_.find(*requested) == highlightedUntilNs_.end() ||
        markers_.find(*requested) == markers_.end())
    {
      requested = requested_.erase(requested);
    } else {
      ++requested;
    }
  }
  const auto wanted = requested_.empty() ? uint64_t{0} : requested_.back();
  {
    // The image the user clicked in is only held while the detection it belongs to stays highlighted.
    std::lock_guard<std::mutex> lock(detectionsMutex_);
    if (pinned_.serial != 0 && pinned_.serial != wanted) {
      pinned_ = PinnedImage();
    }
  }
  if (wanted != selectedSerial_) {
    selectedSerial_ = wanted;
    layoutDirty_ = true;
    layoutUrgent_ = true;  // a click deserves to be answered by the next frame, not by the next layout period
    updateStatus();
  }
  if (!highlightedUntilNs_.empty() && now >= nextSelectionCheckNs_) {
    // Nothing announces the moment a highlight ends, so the view has to be asked to render again now and then. It is
    // only while something is highlighted, and not more often than needed to end the highlight on time.
    nextSelectionCheckNs_ = now + static_cast<int64_t>(SELECTION_REFRESH_PERIOD * 1e9);
    queueRender();
  }
}

void Detection2DArrayDisplay::mousePressed(double x, double y) {
  mousePressed_ = true;
  pressedX_ = x;
  pressedY_ = y;
}

void Detection2DArrayDisplay::clearSelection() {
  requested_.clear();
  highlightedUntilNs_.clear();
  {
    std::lock_guard<std::mutex> lock(detectionsMutex_);
    pinned_ = PinnedImage();
  }
  selectionMissed_ = false;
  selectionGap_ = 1e9;
  if (selectedSerial_ != 0) {
    selectedSerial_ = 0;
    layoutDirty_ = true;
  }
}

void Detection2DArrayDisplay::mouseReleased(double x, double y) {
  const auto pressed = mousePressed_;
  mousePressed_ = false;
  if (!pressed) {
    return;  // the view did not get the press (it lost the focus in between), so this is not a click
  }
  const auto dx = x - pressedX_;
  const auto dy = y - pressedY_;
  if (dx * dx + dy * dy > CLICK_TOLERANCE * CLICK_TOLERANCE) {
    return;  // the mouse moved, so the tool of the view did its work and the user did not click on anything
  }
  selectAt(x, y);
}

bool Detection2DArrayDisplay::selectAt(double x, double y) {
  // The image previews are looked for first: when the user clicks on a preview, they mean the detection it shows, even
  // when the marker of another detection happens to be right below it. The frame and the line to the marker are drawn
  // around the picture, so the place which still counts as the preview includes a margin around it.
  for (const auto& item : items_) {
    if (!item.hasPreview || !item.layout.previewPlaced) {
      continue;
    }
    const auto& rect = item.layout.preview;
    if (x >= rect.x - PREVIEW_MARGIN && x <= rect.x + rect.w + PREVIEW_MARGIN &&
        y >= rect.y - PREVIEW_MARGIN && y <= rect.y + rect.h + PREVIEW_MARGIN)
    {
      select(item.serial);
      return true;
    }
  }
  auto nearest = std::numeric_limits<double>::max();
  const auto best = nearestMarker(x, y, &nearest);
  if (best != 0) {
    select(best);
  } else {
    // What the click led to has to be tellable apart from a click which missed by a few pixels, so the status says how
    // far the closest detection is - that is what tells the two cases from each other.
    selectionMissed_ = true;
    selectionGap_ = nearest;
    updateStatus();
  }
  return best != 0;
}

StorageQuery Detection2DArrayDisplay::historyQuery() const{
  const auto now = nowNs();
  StorageQuery query;
  query.rootDir = storageDirProperty_->getStdString();
  // The period is given as the age of the loaded detections, so that the shown history follows the moving now.
  const auto from = historyFromProperty_->getInt();
  const auto to = historyToProperty_->getInt();
  // A zero limit means no limit, so that the whole storage can be loaded regardless of the (possibly replayed) clock.
  query.startNs = from > 0 ? now - 1000000000LL * static_cast<int64_t>(from) : 0;
  query.endNs = to > 0 ? now - 1000000000LL * static_cast<int64_t>(to) : std::numeric_limits<int64_t>::max();
  // There is no point in reading more detections than the display can show.
  query.limit = static_cast<size_t>(std::max(1, maxMarkersProperty_->getInt()));
  query.zoneFilter = static_cast<int>(historyZoneProperty_->getOptionInt());

  query.classes = splitList(historyClassesProperty_->getStdString());
  query.cameras = splitList(historyCamerasProperty_->getStdString());
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
    detections_.clearHistory();
  }
  history_.records.clear();
  history_.index = 0;
  history_.loaded = 0;
  history_.skipped = 0;
  history_.error.clear();
  history_.running = false;
  historyImages_.clear();
  historyImageOrder_.clear();
  layoutDirty_ = true;
  updateStatus();
}

void Detection2DArrayDisplay::takeHistoryBatch() {
  const auto batchSize = static_cast<size_t>(std::max(1, historyBatchProperty_->getInt()));
  const auto globalFrame = globalFrameProperty_->getStdString();
  const auto minProbability = static_cast<double>(minProbabilityProperty_->getFloat());
  std::vector<StoredDetection> batch;
  batch.reserve(batchSize);
  {
    std::lock_guard<std::mutex> lock(detectionsMutex_);
    while (history_.index < history_.records.size() && batch.size() < batchSize) {
      auto& record = history_.records[history_.index];
      const auto stamp = record.stampNs;
      const auto id = record.id.empty() ? record.detectionId : record.id;
      ++history_.index;
      if (record.probability < minProbability) {
        continue;
      }
      StoredDetection stored;
      stored.stamp = rclcpp::Time(record.stampNs, RCL_ROS_TIME);
      // The detections read from the storage are old; their age is counted from the moment they were recorded, so
      // their "last confirmation" time is their own stamp.
      stored.receivedNs = stamp;
      stored.className = record.className;
      // The camera a stored detection belongs to is the frame of its image; the old storage files know neither.
      stored.cameraFrame = record.cameraFrame.empty() ? record.imageFrame : record.cameraFrame;
      stored.probability = record.probability;
      stored.id = id;
      stored.history = true;
      stored.hasZone = record.zoneStatus == "in" || record.zoneStatus == "out";
      stored.inZone = record.inZone();
      stored.zoneIds = record.zoneIds;
      stored.imagePath = record.imagePath;
      if (record.imageWidth > 0 && record.imageHeight > 0) {
        // The stored image is already the crop around the detection, so it is shown as a whole.
        stored.imageRegion = {0., 0., static_cast<double>(record.imageWidth),
          static_cast<double>(record.imageHeight)};
      }
      stored.previewRegion = stored.imageRegion;
      stored.previewStamp = stored.stamp;
      stored.previewNs = stored.receivedNs;
      stored.previewTaken = true;
      // The global frame is the only one in which the stored positions are comparable between missions, but it can be
      // used only for the detections which were recorded in the frame the user selected.
      record.preferGlobalFrame = !globalFrame.empty() && record.ecefFrame == globalFrame;
      double position[3];
      double orientation[4];
      record.getDisplayPose(position, orientation);
      stored.frameId = record.displayFrame();
      // A detection whose place is known in the global frame has a transform which does not depend on its age - the
      // global frame does not move w.r.t. the Earth. Without this, the transform of a detection recorded days ago would
      // be looked for at its own time, which the transform buffer (holding the last minutes) can never provide.
      stored.inGlobalFrame = !globalFrame.empty() && stored.frameId == globalFrame;
      stored.pose.position.x = position[0];
      stored.pose.position.y = position[1];
      stored.pose.position.z = position[2];
      stored.pose.orientation.x = orientation[0];
      stored.pose.orientation.y = orientation[1];
      stored.pose.orientation.z = orientation[2];
      stored.pose.orientation.w = orientation[3];
      stored.hasPose = record.hasDisplayPose();
      batch.push_back(std::move(stored));
    }
    history_.loaded = history_.index;
    if (!batch.empty()) {
      // The stored detections go through the same filter as the ones received on the topic, so a stored detection
      // which is (almost) at the same place and time as a received one is replaced by it.
      detections_.add(std::move(batch), true);
    }
  }
  layoutDirty_ = true;
  updateStatus();
  queueRender();
}

void Detection2DArrayDisplay::updateHistory(float wall_dt) {
  const auto reloadPeriod = static_cast<double>(historyReloadProperty_->getFloat());
  if (!historyEnabledProperty_->getBool()) {
    if (history_.loaded > 0 || history_.running || !history_.records.empty()) {
      clearHistory();
    }
    history_.age = 0.;
    return;
  }

  bool clockKnown = false;
  nowNs(&clockKnown);
  historyWaitingForClock_ = historyDirty_ && !clockKnown;
  history_.age += wall_dt;
  if (historyDirty_) {
    // Reading the whole storage is the most expensive thing the history does, so it is delayed to make the start of
    // RViz smooth. It also has to wait for the clock of ROS: the period to load is given relative to the current time,
    // with the simulated time of a bag (or of a node which waits for it) the time is zero until the first /clock
    // message arrives - loading then would ask for detections from 1970 and find none.
    const auto delay = static_cast<double>(historyDelayProperty_->getFloat());
    if (history_.age < delay || !clockKnown) {
      updateStatus();
      return;
    }
    historyDirty_ = false;
    clearHistory();
    history_.records = DetectionStore::load(historyQuery(), history_.error, &history_.skipped);
    history_.index = 0;
    history_.running = true;
    history_.pause = 0.;
    history_.age = 0.;
    updateStatus();
    queueRender();
  }

  if (!history_.running) {
    if (reloadPeriod > 0. && history_.age >= reloadPeriod) {
      historyDirty_ = true;
    }
    return;
  }

  // The stored detections are added in batches with pauses between them, so that RViz stays responsive while a long
  // period of history is being shown.
  history_.pause -= wall_dt;
  while (history_.index < history_.records.size() && history_.pause <= 0.) {
    takeHistoryBatch();
    history_.pause += std::max(0., static_cast<double>(historyPauseProperty_->getFloat()));
  }
  if (history_.index >= history_.records.size()) {
    history_.running = false;
    history_.records.clear();
    history_.records.shrink_to_fit();
    history_.age = 0.;
  }
  updateStatus();
}

CameraImage Detection2DArrayDisplay::historyImage(const std::string& path) {
  if (path.empty()) {
    return {};
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
    if (loaded.channels() == 1) {
      encoding = sensor_msgs::image_encodings::MONO8;
    } else if (loaded.channels() == 4) {
      cv::cvtColor(loaded, converted, cv::COLOR_BGRA2BGR);
    }
    const auto& used = converted.empty() ? loaded : converted;
    image.image = std::make_shared<const cv_bridge::CvImage>(std_msgs::msg::Header(), encoding,
        used);
    image.scale = 1.;
  }

  // The images of the stored detections are read from the disk only once and dropped when the cache grows too large.
  historyImageOrder_.push_back(path);
  const auto inserted = historyImages_.insert({path, image}).first;
  while (historyImageOrder_.size() > MAX_HISTORY_IMAGES) {
    const auto oldest = historyImageOrder_.begin();
    historyImages_.erase(*oldest);
    historyImageOrder_.erase(oldest);
  }
  return image;
}

std::string Detection2DArrayDisplay::plural(size_t count, const std::string& word) {
  return std::to_string(count) + ' ' + (count == 1 ? word : word + "s");
}

std::string Detection2DArrayDisplay::secondsText(double seconds) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(seconds < 10. ? 1 : 0) << seconds;
  return stream.str() + " s";
}

std::string Detection2DArrayDisplay::agingText() const {
  const auto lifetime = static_cast<double>(lifetimeProperty_->getFloat());
  const auto inZoneLifetime = static_cast<double>(inZoneLifetimeProperty_->getFloat());
  if (lifetime > 0. && inZoneLifetime > 0.) {
    return secondsText(lifetime) + " out of a keep-out zone and " + secondsText(inZoneLifetime) + " inside one";
  }
  if (lifetime > 0.) {
    return secondsText(lifetime);
  }
  if (inZoneLifetime > 0.) {
    return secondsText(inZoneLifetime) + " inside a keep-out zone";
  }
  return "0 s";
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
    label += "(" + std::to_string(static_cast<int>(std::round(100. * detection.probability))) +
             "%)";
  }
  if (showIdProperty_->getBool() && !detection.id.empty()) {
    if (!label.empty()) {
      label += ' ';
    }
    label += '#' + detection.id;
  }
  // When the detection happened, so that a crowd of detections which stay in the view can be told apart by their age.
  if (showTimeProperty_->getBool() && detection.stamp.nanoseconds() > 0) {
    if (!label.empty()) {
      label += ' ';
    }
    label += formatClock(detection.stamp.nanoseconds());
  }
  // What the zones say about the detection now, falling back to the membership it arrived (or was recorded) with.
  if (showZoneProperty_->getBool() && (detection.inZoneNow || detection.hasZone)) {
    auto zone = insideZone(detection) ? std::string{"zone"} : std::string{"out"};
    const auto& zoneIds = detection.inZoneNow ? detection.nowZoneIds : detection.zoneIds;
    for (size_t i = 0; i < zoneIds.size(); ++i) {
      zone += i == 0 ? " " : ",";
      zone += std::to_string(zoneIds[i]);
    }
    if (!label.empty()) {
      label += ' ';
    }
    label += '[' + zone + ']';
  }
  return label;
}

std::string Detection2DArrayDisplay::previewKey(const StoredDetection& detection) const{
  // The image preview of a detection only changes when the detection itself is forgotten and another one takes its
  // place, or when the detection moved so far that it shows another part of another image. Both are described by the
  // serial number, the stamp of the preview and the shown region.
  return prefix_ + '/' + std::to_string(detection.serial) + '/' +
  std::to_string(detection.previewStamp.nanoseconds()) + '/' +
  std::to_string(std::llround(detection.previewRegion.x)) + '_' +
  std::to_string(std::llround(detection.previewRegion.y)) + '_' +
  std::to_string(std::llround(detection.previewRegion.width)) + '_' +
  std::to_string(std::llround(detection.previewRegion.height));
}

void Detection2DArrayDisplay::layout(Ogre::Camera& camera, Ogre::Viewport& viewport) {
  const auto minProbability = static_cast<double>(minProbabilityProperty_->getFloat());
  const auto markerSize = static_cast<double>(markerSizeProperty_->getFloat());
  const auto fontSize = static_cast<double>(fontSizeProperty_->getFloat());
  const auto spacing = std::max(0., static_cast<double>(spacingProperty_->getFloat()));
  const auto stem = std::max(0., static_cast<double>(stemProperty_->getFloat()));
  // The requested preview size, but never so big that a single preview would take the whole view.
  const auto previewWidth = std::min(static_cast<double>(std::max(8,
      previewWidthProperty_->getInt())),
    .6 * std::min(viewport.getActualWidth(), viewport.getActualHeight()));
  const auto padding = std::max(0., static_cast<double>(previewPaddingProperty_->getFloat()));
  const auto tolerance = static_cast<double>(imageToleranceProperty_->getFloat());
  // Keep the images long enough for the tolerance, plus a margin for the delay of the camera pipeline.
  images_->setKeepDuration(tolerance + .5);
  const auto maxPreviews = showImagesProperty_->getBool() ?
                             static_cast<size_t>(std::max(0, maxPreviewsProperty_->getInt())) : size_t{0};
  const auto maxLabels = static_cast<size_t>(std::max(0, maxLabelsProperty_->getInt()));
  const auto globalFrame = globalFrameProperty_->getStdString();
  // The detection the user asked for is drawn differently from the other ones (see the phases below).
  const auto highlightScale = std::max(1., static_cast<double>(highlightSizeProperty_->getFloat()));
  const auto highlight = highlightColorProperty_->getColor();
  const Ogre::ColourValue highlightColour {static_cast<float>(highlight.redF()),
    static_cast<float>(highlight.greenF()), static_cast<float>(highlight.blueF()), 1.F};

  previewCount_ = 0;
  visualizedCount_ = 0;
  labelCount_ = 0;
  historyCount_ = 0;
  poselessCount_ = 0;
  untransformableCount_ = 0;
  untransformableFrame_.clear();
  closestImageGapNs_ = 0;
  imageCount_ = images_->size();  //!< How many camera images the detections could be matched to.
  cameraFrames();  //!< The cameras the images came from, which the detections have to be matched to.
  refreshAge_ = 0.;
  // The transforms change without any notice, but a static transform (which the one from the global frame is) does not
  // change at all, so the detections are drawn again only once in a while to notice the changes.
  transformRefresh_ = 5.;
  selectedPreviewShown_ = false;
  selectedPreviewNote_.clear();
  PinnedImage pinned;  //!< The image the user clicked in, held for the preview of the detection it highlighted.
  {
    std::lock_guard<std::mutex> lock(detectionsMutex_);
    snapshot_.assign(detections_.detections().begin(), detections_.detections().end());
    pinned = pinned_;
  }

  ScreenMapper mapper {camera, viewport};
  items_.clear();
  if (!mapper.isValid()) {
    clearDrawn();
    return;
  }
  items_.reserve(snapshot_.size());

  // Phase 1: put the detections into the frame of the view and project them. The transform is looked up once per the
  // frame and time of the detections (not once per detection), which is the difference between a millisecond and a
  // second for a dense cluster of detections.
  const auto fixedFrame = context_->getFrameManager()->getFixedFrame();
  struct FrameTransform{
    bool lookedUp {false};  //!< Whether the transform was already looked for.
    bool ok {false};  //!< Whether the transform was found.
    Ogre::Vector3 position {};  //!< Position of the frame in the frame of the view.
    Ogre::Quaternion rotation {};  //!< Rotation of the frame in the frame of the view.
  };
  // The frame of the detection and the time of its transform (zero for a transform which does not depend on time).
  std::map<std::pair<std::string, int64_t>, FrameTransform> transforms;
  for (const auto& detection : snapshot_) {
    if (detection.probability < minProbability) {
      continue;
    }
    if (!detection.hasPose) {
      ++poselessCount_;  // a detection without a 3D position has no place in the 3D scene
      continue;
    }
    const auto& frame = detection.frameId.empty() ? frameId : detection.frameId;
    // The detections which are remembered in the global frame (or which already are in the frame of the view) have a
    // transform which does not depend on their age, because the global frame does not move w.r.t. the Earth. The other
    // detections (their transform to the global frame has not been available when they arrived) have to be transformed
    // by their own time - taking the newest transform for them would make them move around with the sensor they came
    // from, which is exactly what the user must never see.
    const bool timeless = frame.empty() || frame == fixedFrame || detection.inGlobalFrame;
    auto& transform = transforms[std::make_pair(frame,
        timeless ? int64_t{0} : detection.stamp.nanoseconds())];
    if (!transform.lookedUp) {
      transform.lookedUp = true;
      if (frame.empty() || frame == fixedFrame) {
        transform.ok = true;
        transform.rotation = Ogre::Quaternion::IDENTITY;
      } else {
        // Not frameManager->transform() on purpose: it silently falls back to the newest transform when the requested
        // time is not in the TF buffer, which would place an old detection to a place it was not in.
        const auto found = lookupTransform(context_->getFrameManager()->getTransformer(),
            fixedFrame, frame,
            timeless ? latestTime() : detection.stamp);
        if (found) {
          transform.ok = true;
          const auto& origin = found->getOrigin();
          const auto& rotation = found->getRotation();
          transform.position = Ogre::Vector3(origin.x(), origin.y(), origin.z());
          transform.rotation = Ogre::Quaternion(rotation.w(), rotation.x(), rotation.y(),
              rotation.z());
        }
      }
      if (!transform.ok) {
        transformRefresh_ = std::min(transformRefresh_, .5);
        if (untransformableFrame_.empty()) {
          untransformableFrame_ = frame;
        }
      }
    }
    if (!transform.ok) {
      ++untransformableCount_;
      continue;
    }
    const Ogre::Vector3 world {static_cast<float>(detection.pose.position.x),
      static_cast<float>(detection.pose.position.y), static_cast<float>(detection.pose.position.z)};
    Item item;
    ScreenMapper local {mapper};
    local.setReferencePoint(transform.position + transform.rotation * world);
    if (!local.valid()) {
      continue;  // the detection cannot be projected to the view (it is behind the camera)
    }
    item.detection = &detection;
    item.serial = detection.serial;
    item.selected = detection.serial == selectedSerial_;
    item.position = transform.position + transform.rotation * world;
    item.screen = local.centre();
    item.unitX = local.unitX();
    item.unitY = local.unitY();
    item.pixelsPerUnit = local.pixelsPerUnit();
    item.colour = getColor(detection);
    item.label = getLabel(detection);
    // The marker is as large as the user set it, but never smaller than a few pixels, so that the detections far away
    // from the camera stay visible.
    const auto size = std::max(markerSize, MIN_MARKER_SIZE / item.pixelsPerUnit);
    item.radius = .5 * size * item.pixelsPerUnit;
    if (item.selected) {
      // The detection the user asked for has to be found at a glance, so it is drawn bigger and in its own colour.
      item.radius *= highlightScale;
      item.colour = highlightColour;
    }
    // The detections worth looking at come first, and those in the middle of the view before the ones at its border.
    item.priority = std::max(0.,
        std::min(1., detection.probability)) + 1. - local.viewCentreDistance(item.screen);
    if (detection.history) {
      ++historyCount_;
    }
    items_.push_back(std::move(item));
  }
  visualizedCount_ = items_.size();

  // Phase 2: find out what can be shown. Looking a cached image up is cheap, but loading the image of a stored
  // detection from the disk and creating its texture is not, so only the detections which really have a chance at a
  // preview slot get their image prepared - see previewCandidateLimit().
  std::map<uint64_t, size_t> previouslyShown;
  for (size_t i = 0; i < shownPreviews_.size(); ++i) {
    previouslyShown[shownPreviews_[i]] = i;
  }
  const auto storedImages = historyImagesProperty_->getBool();
  std::vector<LayoutItem> annotations(items_.size());
  std::vector<size_t> previewCandidates;
  for (size_t i = 0; i < items_.size(); ++i) {
    auto& item = items_[i];
    auto& annotation = annotations[i];
    const auto& detection = *item.detection;
    const auto marker = std::max(item.radius, 2.);
    item.inView = mapper.insideView(item.screen);
    annotation.anchor = {item.screen.x - marker, item.screen.y - marker, 2. * marker, 2. * marker};
    annotation.priority = item.priority;
    annotation.fresh = knownSerials_.find(item.serial) == knownSerials_.end();
    annotation.hovered = item.serial == hoveredSerial_;
    annotation.selected = item.selected;
    const auto shown = previouslyShown.find(item.serial);
    annotation.wasShown = shown != previouslyShown.end();
    annotation.previousIndex = annotation.wasShown ? static_cast<int>(shown->second) : 0;
    // The width of the line from the preview to the marker is also the space the layout has to keep free for it.
    annotation.stem = stem;
    if (!item.label.empty() && i < maxLabels) {
      annotation.labelWidth = estimateLabelWidth(item.label.length(), fontSize);
      annotation.labelHeight = 1.3 * fontSize;
    }
    if (maxPreviews == 0) {
      continue;
    }
    // Remember which detections could have a preview; their images are only looked up for the most important ones.
    item.textureKey = previewKey(detection);
    // A preview which already has its picture keeps its shape: the layout may not give it a different one, or the
    // picture would look squeezed.
    item.material = textures_->peekMaterial(item.textureKey, &item.textureWidth, &item.textureHeight);
    item.aspectRatio = item.textureWidth > 1 && item.textureHeight > 1 ?
                         static_cast<double>(item.textureWidth) / static_cast<double>(item.textureHeight) : 0.;
    // The highlighted detection is always a candidate for a preview - the user asked to see it, whatever the time
    // tolerance or the images which are cached say about it.
    const bool hasImage = detection.history ? !detection.imagePath.empty() : !imageTopicName_.empty();
    const bool allowed = detection.history ? storedImages : tolerance > 0.;
    if (!item.material.empty() || (hasImage && allowed) || item.selected) {
      previewCandidates.push_back(i);
    }
  }
  std::stable_sort(previewCandidates.begin(), previewCandidates.end(),
      [this, &annotations](size_t a, size_t b) {
        // A detection which is not in the view can never get its preview drawn, so it must not take the place of one
        // which is visible - that is why a preview used to disappear when the view moved away from a dense cluster.
        const auto visible = [this](size_t index) {return items_[index].inView ? 1 : 0;};
        const auto rank = [&annotations](size_t index) {return annotations[index].previewRank();};
        return std::make_tuple(visible(a), rank(a)) > std::make_tuple(visible(b), rank(b));
      });
  previewCandidates.resize(std::min(previewCandidates.size(), previewCandidateLimit(maxPreviews)));
  // The preview is taken from the image of the moment the detection appeared, and once it is created, it stays shown
  // even when that image is long gone from the cache.
  for (const auto i : previewCandidates) {
    auto& item = items_[i];
    auto& annotation = annotations[i];
    const auto& detection = *item.detection;
    // The detections of several cameras can share the topic, and their images can share another one; the frame is the
    // only thing which keeps them apart, so the image of a detection is looked for in its own camera.
    const auto wantedFrameId = wantedFrame(detection.cameraFrame);
    // The image of a live detection is looked up even when its picture was created already: the shape of the preview
    // has to be the shape of the picture in it, and that one follows from the image. The pictures of the stored
    // detections are read from the disk, so they are only read when the picture really has to be created.
    if (!detection.history || item.material.empty()) {
      item.previewImage = detection.history ? historyImage(detection.imagePath) :
                            images_->get(detection.previewStamp, tolerance, wantedFrameId);
      if (!item.previewImage && item.selected && !detection.history) {
        // The highlighted detection is the one the user asked to see, so it is not left without a picture because its
        // image is gone or out of the tolerance: it shows the image the click came from, or the closest one there is.
        item.previewImage = pinned.serial == item.serial && pinned.image ? pinned.image :
                              images_->nearest(detection.previewStamp, wantedFrameId);
        if (!item.previewImage) {
          item.previewImage = images_->nearest(detection.previewStamp);
        }
      }
      if (!item.previewImage) {
        if (!detection.history) {
          // Tell the user how far the camera images really are from the detections, so that they can tell whether the
          // tolerance is too tight or whether no images are arriving at all.
          const auto gap = images_->nearestGapNs(detection.previewStamp, wantedFrameId);
          if (gap >= 0 && (closestImageGapNs_ == 0 || gap < closestImageGapNs_)) {
            closestImageGapNs_ = gap;
          }
        }
        if (item.selected) {
          // Tell the user why the one preview they asked for is missing (see the Selection status).
          if (detection.history) {
            selectedPreviewNote_ = "the picture stored with it cannot be read";
          } else if (imageTopicName_.empty()) {
            selectedPreviewNote_ = "the image topic is not set";
          } else if (images_->size() == 0) {
            selectedPreviewNote_ = "no camera image arrived on it yet";
          } else {
            selectedPreviewNote_ = "no image of the camera it was seen by is in the cache";
          }
        }
        if (item.material.empty()) {
          continue;  // there is nothing to show, neither a picture nor an image to make one from
        }
      }
    }
    auto region = detection.previewRegion.width > 1. && detection.previewRegion.height > 1. ?
                    detection.previewRegion : detection.imageRegion;
    if (region.width <= 1. || region.height <= 1.) {
      region = {0., 0., 0., 0.};  // show the whole image rather than an unknown part of it
    } else if (!detection.history && padding > 1.) {
      region = region.padded(padding);
    }
    item.previewRegion = region;
    // The picture which ends up in the preview is the part of the region which is really inside the image, in the
    // resolution the image is kept in - not the region itself, which often reaches over the border of the image (the
    // padding around a bounding box close to it does that). Only the former shape shows the image without squeezing it.
    if (item.previewImage) {
      const auto source = TextureCache::sourceRect(item.previewImage, region);
      if (source.width > 1 && source.height > 1) {
        item.aspectRatio = static_cast<double>(source.width) / static_cast<double>(source.height);
      } else {
        item.aspectRatio = static_cast<double>(item.previewImage.image->image.cols) /
                           std::max(1, item.previewImage.image->image.rows);
      }
    }
    if (!(item.aspectRatio > 0.)) {
      item.aspectRatio = 4. / 3.;  // neither the image nor a picture of it is known
    }
    item.hasPreview = true;
    annotation.previewWidth = previewWidth;
    annotation.previewHeight = previewWidth / item.aspectRatio;
  }

  // Phase 3: place the annotations so that they do not overlap each other.
  LayoutParams params;
  params.margin = spacing;
  params.maxPreviews = maxPreviews;
  params.maxLabels = maxLabels;
  ScreenLayout plan {mapper.width(), mapper.height(), params};
  plan.place(annotations);
  std::vector<uint64_t> shownNow;
  shownNow.reserve(plan.previewCount());
  for (const auto index : plan.shownPreviews()) {
    shownNow.push_back(items_[index].serial);
  }
  for (size_t i = 0; i < items_.size(); ++i) {
    auto& item = items_[i];
    item.layout.previewPlaced = annotations[i].previewPlaced;
    item.layout.preview = annotations[i].preview;
    item.layout.labelPlaced = annotations[i].labelPlaced;
    item.layout.label = annotations[i].label;
  }
  shownPreviews_.swap(shownNow);

  // Phase 4: the markers. They are real spheres in the scene (the same way the PointStamped display draws a point), so
  // that they shade correctly and can be seen from any view; they are created once per detection and only moved when
  // the detection moved.
  std::set<uint64_t> displayed;
  for (auto& item : items_) {
    displayed.insert(item.serial);
    auto found = markers_.find(item.serial);
    if (found == markers_.end()) {
      auto marker = std::make_unique<Marker>();
      marker->node = scene_node_->createChildSceneNode();
      try {
        marker->entity = scene_manager_->createEntity(prefix_ + "/marker_" +
            std::to_string(item.serial),
            SPHERE_MESH);
        marker->entity->setQueryFlags(0);  // the markers are not pickable
        marker->queue = static_cast<uint8_t>(marker->entity->getRenderQueueGroup());
        marker->node->attachObject(marker->entity);
      } catch (const Ogre::Exception& e) {
        static bool warned {false};
        if (!warned) {
          warned = true;
          RCLCPP_WARN_STREAM(rclcpp::get_logger("cras_rviz_vision_msgs.Detection2DArrayDisplay"),
            "Could not draw the detection markers: " << e.what());
        }
      }
      found = markers_.insert({item.serial, std::move(marker)}).first;
    }
    auto& marker = *found->second;
    if (!item.material.empty()) {
      if (!marker.previewKey.empty() && marker.previewKey != item.textureKey) {
        // The detection shows another image since (it moved, or the user clicked on it in another image), so the
        // texture of the previous one is not needed anymore - it would only take place in the texture cache.
        textures_->release(marker.previewKey);
      }
      marker.previewKey = item.textureKey;
    }
    const auto size = 2. * item.radius / item.pixelsPerUnit;
    if (marker.node) {
      marker.node->setPosition(item.position);
      if (marker.size != size) {
        marker.node->setScale(Ogre::Vector3{static_cast<float>(size), static_cast<float>(size),
              static_cast<float>(size)});
        marker.size = size;
      }
    }
    if (marker.entity) {
      const auto& material = markerMaterial(item.colour, item.selected);
      if (marker.material != material) {
        marker.entity->setMaterialName(material);
        marker.material = material;
      }
      // The highlighted marker is drawn over the other ones, so that it stays visible even in a dense crowd.
      const auto queue = static_cast<uint8_t>(item.selected ? HIGHLIGHT_QUEUE : marker.queue);
      if (queue != marker.entity->getRenderQueueGroup()) {
        marker.entity->setRenderQueueGroup(queue);
      }
    }
  }
  for (auto marker = markers_.begin(); marker != markers_.end();) {
    if (displayed.find(marker->first) != displayed.end()) {
      ++marker;
      continue;
    }
    // The texture of a detection which is not displayed anymore has no chance of being shown again, so it is released
    // (and destroyed later, in the rendering thread).
    if (!marker->second->previewKey.empty()) {
      textures_->release(marker->second->previewKey);
    }
    if (marker->second->entity) {
      scene_manager_->destroyEntity(marker->second->entity);
    }
    if (marker->second->node) {
      scene_manager_->destroySceneNode(marker->second->node);
    }
    marker = markers_.erase(marker);
  }

  // Phase 5: the image previews. They are quads in the plane of the screen, so they always face the camera, and all
  // the previews made from the same texture are drawn by one batch.
  previewObject_->clear();
  annotationObject_->clear();
  const auto baseAlpha = static_cast<double>(previewAlphaProperty_->getFloat());
  const auto drawFrames = previewFrameProperty_->getBool();
  const auto stemWidth = std::max(0., static_cast<double>(stemProperty_->getFloat()));
  std::map<std::string, std::vector<size_t>> byMaterial;
  for (size_t i = 0; i < items_.size(); ++i) {
    auto& item = items_[i];
    if (!item.hasPreview || !item.layout.previewPlaced) {
      continue;
    }
    if (item.material.empty() && item.previewImage) {
      unsigned width = 0;
      unsigned height = 0;
      item.material = textures_->getMaterial(item.previewImage, item.previewRegion,
          static_cast<unsigned>(std::max(1., annotations[i].previewWidth)), item.textureKey, width,
          height);
      if (width > 1 && height > 1) {
        item.textureWidth = width;
        item.textureHeight = height;
        item.aspectRatio = static_cast<double>(width) / static_cast<double>(height);
      }
    }
    if (item.material.empty()) {
      continue;
    }
    byMaterial[item.material].push_back(i);
    ++previewCount_;
    if (item.selected) {
      selectedPreviewShown_ = true;
    }
  }
  // The frame around a preview and the line which leads to its marker tell the previews apart; they all have the colour
  // of their detection and are drawn over everything, so that they never get lost behind a neighbouring preview.
  auto annotationVertex = 0u;  // Number of vertices already handed to the annotation object.
  bool showAnnotations = false;
  if (drawFrames || stemWidth > 0. || selectedSerial_ != 0) {
    // The highlighted detection always gets its frame, even when the frames of the other previews are switched off.
    annotationObject_->begin(annotationMaterial(), Ogre::RenderOperation::OT_TRIANGLE_LIST);
    showAnnotations = true;
  }
  // The preview of the highlighted detection is drawn last of all, so that no other preview can cover the picture the
  // user asked for - the layout may have had to put it over the other previews to have any room for it at all.
  std::string highlighted;  // Material of the highlighted preview, empty when there is none to draw.
  for (const auto& item : items_) {
    if (item.selected && !item.material.empty()) {
      highlighted = item.material;
    }
  }
  std::vector<const std::pair<const std::string, std::vector<size_t>>*> batches;
  batches.reserve(byMaterial.size());
  for (const auto& batch : byMaterial) {
    if (batch.first != highlighted) {
      batches.push_back(&batch);
    }
  }
  const auto last = highlighted.empty() ? byMaterial.end() : byMaterial.find(highlighted);
  if (last != byMaterial.end()) {
    batches.push_back(&*last);
  }
  for (const auto* batch : batches) {
    previewObject_->begin(batch->first, Ogre::RenderOperation::OT_TRIANGLE_LIST);
    auto vertex = 0u;
    for (const auto index : batch->second) {
      auto& item = items_[index];
      if (item.textureWidth > 1 && item.textureHeight > 1) {
        // The room the layout kept for the preview is as close to the shape of the picture as the image allows; if it
        // is not exact, the picture wins - an image is never drawn squeezed, and the frame and the line to the marker
        // are drawn around the same rectangle the preview covers.
        const auto wanted = item.layout.preview.w * static_cast<double>(item.textureHeight) /
                            static_cast<double>(item.textureWidth);
        if (std::abs(wanted - item.layout.preview.h) > 1.) {
          item.layout.preview.y += .5 * (item.layout.preview.h - wanted);
          item.layout.preview.h = wanted;
        }
      }
      const auto& rect = item.layout.preview;
      // The corners are computed in the plane of the screen of the detection, so the preview always faces the camera
      // and has the same scale as the detection next to it.
      const Ogre::Vector2 corners[4] = {
        {static_cast<float>(rect.x), static_cast<float>(rect.y)},
        {static_cast<float>(rect.x + rect.w), static_cast<float>(rect.y)},
        {static_cast<float>(rect.x + rect.w), static_cast<float>(rect.y + rect.h)},
        {static_cast<float>(rect.x), static_cast<float>(rect.y + rect.h)}};
      const Ogre::Vector2 textureCoords[4] = {{0., 0.}, {1., 0.}, {1., 1.}, {0., 1.}};
      // The images have no alpha channel, so their wanted transparency comes from the colour of the corners; the
      // preview the user points at, and the one of the highlighted detection, are opaque so that they can be seen.
      const auto opaque = annotations[index].hovered || item.selected;
      const Ogre::ColourValue tint {1., 1., 1., static_cast<float>(opaque ? 1. : baseAlpha)};
      const auto base = vertex;
      vertex += 4;
      for (int corner = 0; corner < 4; ++corner) {
        previewObject_->position(item.position + (corners[corner].x - item.screen.x) * item.unitX +
            (corners[corner].y - item.screen.y) * item.unitY);
        previewObject_->textureCoord(textureCoords[corner].x, textureCoords[corner].y);
        previewObject_->colour(tint);
      }
      previewObject_->triangle(base, base + 1, base + 2);
      previewObject_->triangle(base, base + 2, base + 3);
      if (showAnnotations) {
        annotatePreview(item, annotations[index].hovered, item.selected, baseAlpha,
          std::max(1., stemWidth) * (annotations[index].hovered ? 2. : 1.5), annotationVertex);
      }
    }
    previewObject_->end();
  }
  if (showAnnotations) {
    annotationObject_->end();
  }
  if (selectedSerial_ != 0 && !selectedPreviewShown_ && selectedPreviewNote_.empty()) {
    // The picture exists, but there was no place to draw it; the layout only gives up when the preview would be a
    // fragment outside of the view.
    selectedPreviewNote_ = "no place to show it in the view was found";
  }

  // Phase 6: the text labels. They are drawn by a persistent text object which is passed from one detection to
  // another, so that showing a label does not mean creating one.
  const auto orientation = camera.getDerivedOrientation().Inverse();
  if (!textFailed_) {
    std::map<uint64_t, size_t> usedLabels;
    for (auto& item : items_) {
      if (item.label.empty() || !item.layout.labelPlaced) {
        continue;
      }
      auto known = labelOf_.find(item.serial);
      if (known == labelOf_.end()) {
        auto free = labels_.end();
        for (auto label = labels_.begin(); label != labels_.end(); ++label) {
          if ((*label)->serial == 0) {
            free = label;
            break;
          }
        }
        if (free == labels_.end()) {
          if (labels_.size() >= maxLabels) {
            break;  // the labels of the remaining detections did not fit into the limit
          }
          labels_.push_back(std::make_unique<Label>());
          free = std::prev(labels_.end());
          if (!createLabelText(*labels_.back())) {
            break;
          }
        }
        (*free)->serial = item.serial;
        known = labelOf_.insert({item.serial,
              static_cast<size_t>(std::distance(labels_.begin(), free))}).first;
      }
      auto& label = *labels_[known->second];
      if (label.caption != item.label) {
        label.text->setCaption(item.label);
        label.caption = item.label;
      }
      // The text has to be as tall in the world as the font is in the view, so that the labels keep their size when
      // the user zooms in and out - the same what the layout counts with when it reserves room for them.
      const auto height = static_cast<float>(fontSize / item.pixelsPerUnit);
      if (label.height != height) {
        label.text->setCharacterHeight(height);
        label.height = height;
      }
      label.text->setColor(item.colour);
      // The text is written up and to the right from its position (see setTextAlignment below), while the layout
      // counted with a box which starts in its top-left corner, so the position is near the bottom of that box.
      const auto& rect = item.layout.label;
      const auto place = item.position + static_cast<float>(rect.x - item.screen.x) * item.unitX +
                         static_cast<float>(rect.y + .85 * rect.h - item.screen.y) * item.unitY;
      label.node->setPosition(place);
      // The text builds its geometry only when it is asked to, and nothing else notices that its caption or size
      // changed, so without this call the labels would stay empty (and therefore invisible).
      label.text->update();
      // The text is written in the XY plane of its node, so the node has to turn with the camera - otherwise the labels
      // are seen from the side (and in a top-down view they are edge-on and invisible).
      label.node->setOrientation(orientation);
      ignoreDepth(*label.text);  // the material is shared with nothing, but setting it again costs nothing
      label.text->setVisible(true);
      usedLabels[item.serial] = known->second;
      ++labelCount_;
    }
    for (auto label = labelOf_.begin(); label != labelOf_.end();) {
      if (usedLabels.find(label->first) == usedLabels.end()) {
        labels_[label->second]->serial = 0;
        labels_[label->second]->text->setVisible(false);
        label = labelOf_.erase(label);
      } else {
        ++label;
      }
    }
  }

  knownSerials_.clear();
  knownSerials_.reserve(items_.size());
  for (const auto& item : items_) {
    knownSerials_.insert(item.serial);
  }
  queueRender();
}

void Detection2DArrayDisplay::ignoreDepth(rviz_rendering::MovableText& text) {
  const auto& material = text.getMaterial();
  material->setDepthCheckEnabled(false);
  material->setDepthWriteEnabled(false);
  text.showOnTop(true);
}

bool Detection2DArrayDisplay::createLabelText(Label& label) {
  try {
    label.node = overlayNode_->createChildSceneNode();
    label.text = std::make_unique<rviz_rendering::MovableText>("", fontProperty_->getString().toStdString(),
      static_cast<float>(fontSizeProperty_->getFloat()));
    // The text is owned by the label; the node only draws it.
    label.text->setQueryFlags(0);  // the annotations are not pickable
    label.text->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY + 1);  // over the image previews and their frames
    // The text material checks the depth by default, and the labels are at the same depth as the image previews and
    // the markers they belong to - without turning the depth check off, the previews would hide the labels.
    ignoreDepth(*label.text);
    label.text->setTextAlignment(rviz_rendering::MovableText::H_LEFT,
        rviz_rendering::MovableText::V_ABOVE);
    label.node->attachObject(label.text.get());
    return true;
  } catch (const Ogre::Exception& e) {
    textFailed_ = true;
    RCLCPP_WARN_STREAM(rclcpp::get_logger("cras_rviz_vision_msgs.Detection2DArrayDisplay"),
      "Could not render the detection labels: " << e.what());
    label.node = nullptr;
    label.text.reset();
    return false;
  }
}

void Detection2DArrayDisplay::report(
    const std::string& name,
    const rviz_common::properties::StatusProperty::Level level, const std::string& message) {
  // The status is recalculated in every frame, so that a warning disappears as soon as the condition which caused it
  // is fixed. RViz keeps a status message until it is changed or deleted, and creating the status properties again in
  // every frame would be slow, so only the changes are passed on.
  const auto value = static_cast<int>(level);
  const auto known = lastStatus_.find(name);
  if (known != lastStatus_.end() && known->second.first == value &&
      known->second.second == message)
  {
    return;
  }
  lastStatus_[name] = {value, message};
  if (message.empty()) {
    deleteStatusStd(name);
    return;
  }
  setStatusStd(level, name, message);
  if (statusDebug_ && level == rviz_common::properties::StatusProperty::Ok) {
    // The informational messages change with every message which arrives, so they are reported only once in a while.
    const auto logged = lastStatusLogNs_.find(name);
    const auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    if (logged == lastStatusLogNs_.end() ||
        now - logged->second > static_cast<int64_t>(STATUS_LOG_PERIOD * 1e9))
    {
      lastStatusLogNs_[name] = now;
      RCLCPP_INFO_STREAM(rclcpp::get_logger("cras_rviz_vision_msgs.Detection2DArrayDisplay"),
        name << ": " << message);
    }
  }
  if (level != rviz_common::properties::StatusProperty::Ok) {
    // Also tell it in the log, so that a problem is not hidden when nobody looks at the panel (e.g. when RViz runs in
    // the background or inside another application). This happens only when the message changes, not every frame.
    RCLCPP_WARN_STREAM(rclcpp::get_logger("cras_rviz_vision_msgs.Detection2DArrayDisplay"),
        name << ": " << message);
  }
}

void Detection2DArrayDisplay::updateStatus() {
  using Level = rviz_common::properties::StatusProperty::Level;
  if (!images_) {
    return;  // the display is not initialized yet, the status properties do not exist either
  }
  size_t total = 0;
  size_t dropped = 0;
  size_t expired = 0;
  size_t merged = 0;
  std::string zoneFrame;
  {
    std::lock_guard<std::mutex> lock(detectionsMutex_);
    total = detections_.size();
    dropped = detections_.dropped();
    expired = detections_.expired();
    merged = detections_.replaced();
    zoneFrame = zones_.getFrame();
  }
  const auto remember = static_cast<size_t>(std::max(1, maxMarkersProperty_->getInt()));
  const auto maxPreviews = static_cast<size_t>(std::max(0, maxPreviewsProperty_->getInt()));
  const auto lifetime = static_cast<double>(lifetimeProperty_->getFloat());
  const auto minProbability = static_cast<double>(minProbabilityProperty_->getFloat());

  if (total == 0) {
    if (messagesReceived_ == 0) {
      report("Detections", Level::Ok,
          "Waiting for a message on the topic " + topic_property_->getTopicStd() + '.');
    } else {
      // The message rate of RViz is an average since the subscription was created, which is misleading right after the
      // start, so report the rate which the display measured itself.
      std::ostringstream count;
      count << messagesReceived_;
      std::ostringstream rate;
      rate << std::fixed << std::setprecision(1) << messageRate_;
      if (expired > 0) {
        // The messages did contain detections, the display only forgot them again - which is what the lifetimes are
        // for, but the user should be able to see it when nothing is left to show.
        report("Detections", Level::Ok, "All the detections which arrived on " +
          topic_property_->getTopicStd() + " have run out of their age (" + agingText() + "); the topic sent " +
          count.str() + " messages (" + rate.str() + " per s).");
      } else {
        report("Detections", Level::Ok, "The " + count.str() + " messages which arrived on " +
          topic_property_->getTopicStd() + " (" + rate.str() + " per s) contained no detection.");
      }
    }
  } else if (visualizedCount_ == 0) {
    // The reason why nothing is drawn is said by the other aspects of the status (the poses, the transforms).
    report("Detections", Level::Warn,
        "None of the " + std::to_string(total) + " remembered detections can be "
      "shown.");
  } else {
    auto message = "Showing " + std::to_string(visualizedCount_) + " of " + std::to_string(total) +
                   " detections";
    if (historyCount_ > 0) {
      message += " (" + std::to_string(historyCount_) + " of them from the storage)";
    }
    message += ", " + std::to_string(previewCount_) + " of them with an image preview and " +
               plural(labelCount_, "label") + '.';
    if (filteredCount_ > 0) {
      std::ostringstream threshold;
      threshold << std::fixed << std::setprecision(2) << minProbability;
      message += " " + std::to_string(filteredCount_) +
                 " detections were hidden because their probability is below " +
                 threshold.str() + '.';
    }
    if (merged > 0) {
      message += " " + std::to_string(merged) + " reports of the same object were merged.";
    }
    if (dropped > 0 || expired > 0) {
      // Forgetting old detections is what the user asked for, so it is only said when it really starts to bite.
      message += " " + std::to_string(dropped + expired) + " older detections were forgotten";
      if (dropped > 0) {
        message += " to keep their number below " + std::to_string(remember);
      }
      if (expired > 0) {
        message += (dropped > 0 ? " and their age below " : " because they were older than ") + agingText();
      }
      message += '.';
      if (dropped > 0) {
        report("Detections", Level::Warn,
            message + " Consider merging the detections which are close together, or "
          "showing more of them.");
      } else {
        report("Detections", Level::Ok, message);
      }
    } else {
      report("Detections", Level::Ok, message);
    }
  }

  if (poselessCount_ > 0 && poselessCount_ == total) {
    report("3D poses", Level::Error,
        "None of the detections has a 3D pose in results[].pose.pose, so it cannot be "
      "placed in the scene. Only a bounding box in the image is known for them.");
  } else if (poselessCount_ > 0) {
    report("3D poses", Level::Warn,
        std::to_string(poselessCount_) + " detections have no 3D pose in "
      "results[].pose.pose and cannot be placed in the scene.");
  } else {
    report("3D poses", Level::Ok, {});
  }

  const auto untransformable = untransformableCount_;
  if (untransformable > 0 && visualizedCount_ == 0) {
    report("Transforms", Level::Error,
        "The frame " + untransformableFrame_ + " cannot be transformed to " +
      context_->getFrameManager()->getFixedFrame() +
        ", so nothing can be shown. Check that the TF tree is being "
      "published; note that a static transform may only appear some time after the system starts.");
  } else if (untransformable > 0) {
    report("Transforms", Level::Warn, std::to_string(untransformable) + " detections in frame " +
      untransformableFrame_ + " cannot be transformed to " +
        context_->getFrameManager()->getFixedFrame() +
      ". They are kept in their original frame and the display keeps looking for the transform, so they can appear "
      "later.");
  } else {
    report("Transforms", Level::Ok, {});
  }

  // The keep-out zones are an optional input, but the display needs them for everything which tells a detection inside
  // a zone from one which is out of them - the colour by the zone, and the two different lifetimes.
  const auto zonesTopic = zonesTopicProperty_->getTopicStd();
  const auto inZoneLifetime = static_cast<double>(inZoneLifetimeProperty_->getFloat());
  if (zonesTopic.empty()) {
    report("Keep-out zones", Level::Ok, {});
  } else if (zoneCount_ == 0) {
    // Without the zones there is nothing to tell the two lifetimes apart, only the one for the detections out of them
    // is applied to everything.
    auto message = "No keep-out zone is published on " + zonesTopic + ", so every detection counts as being out of "
                   "them";
    if (lifetime > 0.) {
      message += " and expires after " + secondsText(lifetime);
    }
    if (inZoneLifetime > 0.) {
      message += ", and the lifetime of " + secondsText(inZoneLifetime) +
                 " for the detections inside a zone has nothing to apply to";
    }
    report("Keep-out zones", inZoneLifetime > 0. ? Level::Warn : Level::Ok, message + '.');
  } else {
    auto message = "The display knows " + plural(zoneCount_, "keep-out zone") + " in frame " +
                   (zoneFrame.empty() ? "(no frame)" : zoneFrame) + "; it found " + std::to_string(inZoneCount_) +
                   " of the remembered detections in one of them";
    if (unknownZoneCount_ > 0) {
      message += ", and it is not known for " + std::to_string(unknownZoneCount_) +
                 " of them (their position in the frame of the zones cannot be found)";
    }
    report("Keep-out zones", Level::Ok, message + '.');
  }

  // A display without an image topic and without anything from the storage can have no previews for a simple reason.
  const auto noImagesAtAll = imageTopicName_.empty() && historyCount_ == 0;
  if (!showImagesProperty_->getBool() || maxPreviews == 0) {
    report("Image previews", Level::Ok, {});
  } else if (!images_->lastError().empty()) {
    report("Image previews", Level::Error, images_->lastError());
  } else if (previewCount_ == 0 && visualizedCount_ > 0 && noImagesAtAll) {
    report("Image previews", Level::Warn,
        "No camera image topic is set, so the detections can only be shown with "
      "their marker and label.");
  } else if (previewCount_ == 0 && visualizedCount_ > 0 && imageCount_ == 0) {
    report("Image previews", Level::Warn,
        "No camera image arrived on " + imageTopicName_ + " yet, so the detections "
      "can only be shown with their marker and label.");
  } else if (previewCount_ == 0 && visualizedCount_ > 0) {
    // The most common reason is a clock or latency mismatch of the image and detection stamps, so say what the actual
    // distance of the two is. And when the images come from several cameras, say which of them the display sees - a
    // detection of a camera whose images are not arriving can never find its image.
    std::string cameras;
    for (const auto& source : images_->sources()) {
      if (!cameras.empty()) {
        cameras += ", ";
      }
      cameras += (source.frameId.empty() ? "(no frame)" : source.frameId) + ": " +
                 std::to_string(source.images) + " images";
      if (!source.error.empty()) {
        cameras += ", cannot be converted";
      }
    }
    auto message = "None of the cached camera images (" + cameras +
                   ") is close enough to the detections";
    if (closestImageGapNs_ > 0) {
      // The distance of the stamps grows with every message, so the number is reported only roughly - otherwise the
      // status would change on every frame and the panel of the display would flicker uselessly.
      message += "; the closest one is about " + formatTime(closestImageGapNs_) + " away";
    }
    message += " (the tolerance is " +
               std::to_string(static_cast<int>(1000. * imageToleranceProperty_->getFloat())) +
               " ms). Either raise the tolerance, or make sure the detections are stamped by the camera time.";
    report("Image previews", Level::Warn, message);
  } else if (previewCount_ < std::min(maxPreviews, visualizedCount_)) {
    report("Image previews", Level::Ok, std::to_string(previewCount_) + " image previews are shown; the rest of the "
      "detections had no room for one (zooming in makes room for more of them).");
  } else {
    report("Image previews", Level::Ok, {});
  }

  if (!historyEnabledProperty_->getBool()) {
    report("Persistent history", Level::Ok, {});
  } else if (history_.running) {
    report("Persistent history", Level::Ok,
        "Loading the stored detections: " + std::to_string(history_.index) +
      " of " + std::to_string(history_.records.size()) + " added (" +
        std::to_string(history_.loaded) + " of them "
      "are displayed).");
  } else if (historyWaitingForClock_) {
    report("Persistent history", Level::Warn,
      "Waiting for the clock of ROS to know which detections to load. The period of the query is relative to the "
      "current time, and with the simulated time no time has passed until the first /clock message arrives.");
  } else if (historyDirty_) {
    const auto delay = static_cast<double>(historyDelayProperty_->getFloat());
    report("Persistent history", Level::Ok, "The stored detections will be loaded in " +
      std::to_string(static_cast<int>(std::max(0., delay - history_.age))) + " s.");
  } else if (!history_.error.empty()) {
    report("Persistent history", Level::Error, history_.error);
  } else if (history_.skipped > 0) {
    report("Persistent history", Level::Warn,
        std::to_string(history_.skipped) + " rows of the storage could not be "
      "read.");
  } else if (history_.loaded > 0 && historyCount_ == 0 && lifetimeProperty_->getFloat() > 0.) {
    // The stored detections are not exempt from the timeouts, so a storage which is older than they allow is empty in
    // the view and the user would only see nothing happening.
    report("Persistent history", Level::Warn, "Loaded " + std::to_string(history_.loaded) +
      " detections from " + storageDirProperty_->getStdString() +
      ", but all of them are older than the allowed age, so none of them is shown. Either raise Max. age, or ask for a "
      "shorter period of the storage.");
  } else {
    // What was read from the storage and what is actually drawn are two different things - the second one is told by
    // the status of the detections and of the transforms, so say at least how many of the loaded ones are displayed.
    report("Persistent history", Level::Ok, "Loaded " + std::to_string(history_.loaded) + " detections from " +
      storageDirProperty_->getStdString() + ", " + std::to_string(historyCount_) + " of them are displayed.");
  }

  if (selectionMissed_) {
    report("Selection", Level::Warn,
        "The place the user clicked does not belong to any of the shown detections" +
        (selectionGap_ < 1e6 ? "; the closest marker is about " +
           std::to_string(static_cast<int>(coarseDistance(selectionGap_))) + " px away" : "") +
      ". A detection is highlighted when its bounding box contains the clicked place (or is close to it), its camera "
      "matches the clicked image and their stamps differ by at most " +
      std::to_string(static_cast<int>(1000. * selectionToleranceProperty_->getFloat())) + " ms.");
  } else if (selectedSerial_ != 0) {
    // One message for the whole time the detection is highlighted - only a preview which cannot be shown is worth
    // saying, because that is exactly what the user clicked for.
    report("Selection", !selectedPreviewShown_ && !selectedPreviewNote_.empty() ? Level::Warn : Level::Ok,
        "The clicked detection is highlighted: its marker is drawn above the other ones, and its image preview is "
      "opaque, framed, drawn over the other previews and the first one to get a place in the layout, for "
        +
      std::to_string(static_cast<int>(highlightProperty_->getFloat())) + " s." +
        (selectedPreviewShown_ || selectedPreviewNote_.empty() ? std::string() :
           " Its preview cannot be shown: " + selectedPreviewNote_ + "."));
  } else if (!selectionTopicName_.empty()) {
    report("Selection", Level::Ok,
        "Waiting for a click on a camera image on " + selectionTopicName_ +
      ". Clicking a marker or an image preview in the view highlights the detection too.");
  } else {
    report("Selection", Level::Ok, {});
  }
}

Ogre::Vector3 Detection2DArrayDisplay::screenPoint(const Item& item, double x, double y) {
  const auto offset = static_cast<float>(x - item.screen.x) * item.unitX +
                      static_cast<float>(y - item.screen.y) * item.unitY;
  return item.position + offset;
}

void Detection2DArrayDisplay::addAnnotationQuad(
    const Ogre::Vector3& a, const Ogre::Vector3& b,
    const Ogre::Vector3& c, const Ogre::Vector3& d, const Ogre::ColourValue& colour,
    unsigned& vertex) {
  annotationObject_->position(a);
  annotationObject_->colour(colour);
  annotationObject_->position(b);
  annotationObject_->colour(colour);
  annotationObject_->position(c);
  annotationObject_->colour(colour);
  annotationObject_->position(d);
  annotationObject_->colour(colour);
  annotationObject_->triangle(vertex, vertex + 1, vertex + 2);
  annotationObject_->triangle(vertex, vertex + 2, vertex + 3);
  vertex += 4;
}

void Detection2DArrayDisplay::annotatePreview(
    Item& item, bool hovered, bool selected, double alpha,
    double thickness, unsigned& vertex) {
  auto colour = item.colour;
  if (selected) {
    // The frame of the preview of a highlighted detection is the colour the user set for it (which is the colour of its
    // marker too), it is always drawn, and it is thicker than the frames of the other previews.
    thickness *= 1.5;
  } else if (hovered) {
    // The preview the user points at stands out by being brighter and thicker.
    colour = Ogre::ColourValue(std::min(1.F, .45F + colour.r), std::min(1.F, .45F + colour.g),
        std::min(1.F, .45F + colour.b), 1.F);
  } else {
    colour.a = static_cast<float>(std::max(.8, alpha));
  }
  const auto& rect = item.layout.preview;
  if (selected || previewFrameProperty_->getBool()) {
    // The four sides of the frame are quads, so that they have the same thickness in any view.
    const auto t = thickness;
    addAnnotationQuad(screenPoint(item, rect.x - t, rect.y - t),
        screenPoint(item, rect.x + rect.w + t, rect.y - t),
        screenPoint(item, rect.x + rect.w, rect.y),
        screenPoint(item, rect.x, rect.y), colour, vertex);
    addAnnotationQuad(screenPoint(item, rect.x, rect.y + rect.h),
        screenPoint(item, rect.x + rect.w, rect.y + rect.h),
        screenPoint(item, rect.x + rect.w + t, rect.y + rect.h + t),
        screenPoint(item, rect.x - t, rect.y + rect.h + t), colour, vertex);
    addAnnotationQuad(screenPoint(item, rect.x - t, rect.y), screenPoint(item, rect.x, rect.y),
        screenPoint(item, rect.x, rect.y + rect.h),
        screenPoint(item, rect.x - t, rect.y + rect.h + t),
        colour, vertex);
    addAnnotationQuad(screenPoint(item, rect.x + rect.w, rect.y),
        screenPoint(item, rect.x + rect.w + t, rect.y),
        screenPoint(item, rect.x + rect.w + t, rect.y + rect.h + t),
        screenPoint(item, rect.x + rect.w, rect.y + rect.h), colour, vertex);
  }
  const auto stemWidth = std::max(0., static_cast<double>(stemProperty_->getFloat()));
  if (stemWidth <= 0.) {
    return;
  }
  // The line starts at the marker and ends at the closest point of the preview, which is the shortest way between them
  // and therefore the least confusing one in a crowded scene.
  const auto markerX = static_cast<double>(item.screen.x);
  const auto markerY = static_cast<double>(item.screen.y);
  const auto borderX = std::max(rect.x, std::min(markerX, rect.x + rect.w));
  const auto borderY = std::max(rect.y, std::min(markerY, rect.y + rect.h));
  const auto directionX = borderX - markerX;
  const auto directionY = borderY - markerY;
  const auto length = std::sqrt(directionX * directionX + directionY * directionY);
  if (length <= 2. * thickness) {
    return;  // the marker is right at the preview, no line is needed to say they belong together
  }
  const auto halfX = .5 * thickness * directionY / length;
  const auto halfY = -.5 * thickness * directionX / length;
  addAnnotationQuad(screenPoint(item, markerX - halfX, markerY - halfY),
      screenPoint(item, markerX + halfX, markerY + halfY),
      screenPoint(item, borderX + halfX, borderY + halfY),
      screenPoint(item, borderX - halfX, borderY - halfY), colour, vertex);
}

size_t Detection2DArrayDisplay::getVisualizationSize() const{
  return visualizedCount_;
}

size_t Detection2DArrayDisplay::getPreviewCount() const{
  return previewCount_;
}

size_t Detection2DArrayDisplay::getHistoryCount() const{
  return historyCount_;
}

uint64_t Detection2DArrayDisplay::getSelectedSerial() const{
  return selectedSerial_;
}

void Detection2DArrayDisplay::redraw() {
  layoutDirty_ = true;
  updateStatus();
  queueRender();
}

}  // namespace cras_rviz_vision_msgs

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(cras_rviz_vision_msgs::Detection2DArrayDisplay, rviz_common::Display)
