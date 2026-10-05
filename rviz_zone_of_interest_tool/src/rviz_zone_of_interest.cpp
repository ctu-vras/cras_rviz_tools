// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <Eigen/Dense>
#include <OgreCamera.h>
#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <polygon_msgs/msg/polygon2_d_collection.hpp>
#include <QKeyEvent>
#include <rclcpp/qos.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/interaction/view_picker.hpp>
#include <rviz_common/load_resource.hpp>
#include <rviz_common/logging.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/qos_profile_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/view_controller.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_rendering/material_manager.hpp>
#include <rviz_rendering/objects/movable_text.hpp>
#include <rviz_rendering/viewport_projection_finder.hpp>
#include <rviz_zone_of_interest_tool/rviz_zone_of_interest_tool.hpp>

/**
 * \brief Create an orthonormal matrix whose third column points in the given direction.
 *
 * The remaining two axes are chosen arbitrarily in the plane perpendicular to the given normal.
 *
 * \param[in] norm The direction of the third column of the matrix. It is expected to be normalized.
 * \return The rotation matrix with \p norm as its third column.
 */
static Eigen::Matrix3f createMatrix(const Eigen::Vector3f& norm) {
  Eigen::Matrix3f mat(Eigen::Matrix3f::Identity());
  mat.col(2) = norm;

  // Create plane from point normal
  Eigen::Hyperplane<float, 3> plane(norm, Eigen::Vector3f(0, 0, 0));

  // If the normal and global x-axis are not closely aligned
  if (std::abs(norm.dot(Eigen::Vector3f::UnitX())) < 0.90f) {
    // Project the global x-axis onto the plane to generate the x-axis
    Eigen::Vector3f x_axis = plane.projection(Eigen::Vector3f::UnitX()).normalized();
    mat.col(0) = x_axis;
    mat.col(1) = norm.cross(x_axis);
  } else {
    // Project the global y-axis onto the plane to generate the y-axis
    Eigen::Vector3f y_axis = plane.projection(Eigen::Vector3f::UnitY()).normalized();
    mat.col(0) = y_axis.cross(norm);
    mat.col(1) = y_axis;
  }

  return mat;
}

/**
 * \brief Estimate the orientation of the plane that best fits the given points.
 *
 * Principal component analysis is used to find the direction with the smallest variance, which is taken as the
 * normal of the (approximately planar) point selection. The normal is flipped so that it faces the camera.
 *
 * \param[in] points The points the plane shall be estimated from.
 * \param[in] camera_norm The direction the camera is looking in.
 * \return The orientation whose Z-axis is the estimated plane normal.
 */
static Ogre::Quaternion estimateNormal(const std::vector<Ogre::Vector3>& points, const Ogre::Vector3& camera_norm) {
  Eigen::MatrixXf data;
  data.resize(points.size(), 3);

  for (std::size_t i = 0; i < points.size(); ++i) {
    data.row(i) = Eigen::Map<const Eigen::Vector3f>(points[i].ptr());
  }

  Eigen::MatrixXf centered = data.rowwise() - data.colwise().mean();

  // Use principal component analysis to the get eigenvectors
  Eigen::MatrixXf cov = centered.transpose() * centered;
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eig(cov);

  // Get the eigenvector associated with the smallest eigenvalue
  // (should be Z-axis, assuming points are relatively planar)
  Eigen::Vector3f norm = eig.eigenvectors().col(0);
  norm.normalize();

  Eigen::Vector3f camera_normal;
  camera_normal << camera_norm.x, camera_norm.y, camera_norm.z;
  camera_normal.normalize();

  if (norm.dot(camera_normal) < 0) {
    norm *= -1;
  }

  // Create an arbitrary orientation matrix with the normal being in the direction of the smallest eigenvector
  Eigen::Matrix3f mat = createMatrix(norm);

  Eigen::Quaternionf q(mat);  // Eigen::AngleAxisf(0.0, evecs.col(2)));
  Ogre::Quaternion out;
  out.w = q.w();
  out.x = q.x();
  out.y = q.y();
  out.z = q.z();

  return out;
}

namespace rviz_zone_of_interest_tool {

rclcpp::QoS ZoneOfInterestTool::defaultQosProfile() {
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  // The polygons describe a state, not a stream of updates - keep them latched for late subscribers
  qos.transient_local();

  return qos;
}

void ZoneOfInterestTool::updateMaterialColor(
    Ogre::MaterialPtr material, const QColor& color, const bool override_self_illumination) {
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
  float r, g, b, a;
#else
  qreal r, g, b, a;
#endif
  color.getRgbF(&r, &g, &b, &a);
  material->setDiffuse(r, g, b, a);
  material->setSpecular(r, g, b, a);
  material->setAmbient(r, g, b);
  if (override_self_illumination) {
    material->setSelfIllumination(r, g, b);
  }
}

ZoneOfInterestTool::ZoneOfInterestTool()
    : rviz_common::Tool(), qos_profile_(defaultQosProfile()) {
  shortcut_key_ = 'z';
}

ZoneOfInterestTool::~ZoneOfInterestTool() {
  // Remove materials
  Ogre::MaterialManager::getSingleton().remove(points_material_);
  Ogre::MaterialManager::getSingleton().remove(lines_material_);

  // The QoS subproperties themselves are owned by the topic property, only this wrapper has to be deleted here
  delete qos_profile_property_;

  // Remove displays
  removeDisplays();

  // Remove top level scene nodes
  scene_manager_->getRootSceneNode()->removeAndDestroyChild(points_node_);
  scene_manager_->getRootSceneNode()->removeAndDestroyChild(lines_node_);
  scene_manager_->getRootSceneNode()->removeAndDestroyChild(text_node_);

  if (cursor_node_->getParentSceneNode()) {
    cursor_node_->getParentSceneNode()->removeChild(cursor_node_);
  }
  scene_manager_->destroySceneNode(cursor_node_);
}

void ZoneOfInterestTool::onInitialize() {
  // Add the properties
  topic_property_ = new rviz_common::properties::RosTopicProperty(
    "Topic name", "polygons",
    QString::fromStdString(rosidl_generator_traits::name<polygon_msgs::msg::Polygon2DCollection>()),
    "The latched topic to publish the finished polygons on", getPropertyContainer(), SLOT(updateTopicName()), this);

  // Adds the QoS subproperties (depth, history, reliability and durability policies) to the topic property
  qos_profile_property_ = new rviz_common::properties::QosProfileProperty(topic_property_, qos_profile_);

  pt_color_property_ = new rviz_common::properties::ColorProperty(
    "Point Color", Qt::black, "Color of the points", getPropertyContainer(), SLOT(updatePointsColor()), this);

  line_color_property_ = new rviz_common::properties::ColorProperty(
    "Line Color", Qt::black, "Color of the line", getPropertyContainer(), SLOT(updateLinesColor()), this);

  pt_size_property_ = new rviz_common::properties::FloatProperty(
    "Point Size", 5.0, "Size of clicked points", getPropertyContainer(), SLOT(updatePointsSize()), this);

  text_visibility_property_ =
    new rviz_common::properties::BoolProperty(
      "Show Text", true, "Toggles the visibility of the text display",
      getPropertyContainer(), SLOT(updateTextVisibility()), this);

  text_size_property_ = new rviz_common::properties::FloatProperty(
    "Text Size", 0.015, "Height of the text display (m)", getPropertyContainer(), SLOT(updateTextSize()), this);

  render_as_overlay_property_ = new rviz_common::properties::BoolProperty(
    "Render as Overlay", false, "Render the polygon points and lines on top of all other geometry",
    getPropertyContainer(), SLOT(updateRenderAsOverlay()), this);

  projection_finder_ = std::make_shared<rviz_rendering::ViewportProjectionFinder>();

  // Configure the ROS topic interface
  topic_property_->initialize(context_->getRosNodeAbstraction());
  qos_profile_property_->initialize(
      [this](rclcpp::QoS updated_qos) {
        qos_profile_ = updated_qos;
        updateTopicName();
      });
  updateTopicName();

  // Configure the displays
  static int count = 0;
  points_material_ =
    rviz_rendering::MaterialManager::createMaterialWithLighting("points_material_" + std::to_string(count));
  lines_material_ =
    rviz_rendering::MaterialManager::createMaterialWithLighting("lines_material_" + std::to_string(count));
  ++count;

  points_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  lines_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  text_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  newPolygon();

  // Initialize the scene node
  cursor_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  // Create the visual tool object
  cursor_object_ = createToolVisualization();

  // Attach the tool visualization to the scene
  cursor_node_->attachObject(cursor_object_);
  cursor_node_->setVisible(false);

  // Set the cursors
  hit_cursor_ = cursor_;
  std_cursor_ = rviz_common::getDefaultCursor();
}

void ZoneOfInterestTool::activate() {
  cursor_node_->setVisible(true);
}

void ZoneOfInterestTool::deactivate() {
  updateText();
  cursor_node_->setVisible(false);
}

void ZoneOfInterestTool::newPolygon() {
  // Add a new empty selection vector
  points_.emplace_back();

  // Add the points visualization
  Ogre::ManualObject* points = scene_manager_->createManualObject();
  points_node_->attachObject(points);

  // Add the lines visualization
  Ogre::ManualObject* lines = scene_manager_->createManualObject();
  lines_node_->attachObject(lines);

  // Add the text
  std::string caption = "#" + std::to_string(points_.size());
  auto* text = new rviz_rendering::MovableText(caption, "Liberation Sans", text_size_property_->getFloat(),
                                               Ogre::ColourValue::Blue);
  text->setVisible(false);
  text->setTextAlignment(rviz_rendering::MovableText::H_CENTER, rviz_rendering::MovableText::V_ABOVE);

  // Attach the text to a movable child scene node underneath the main text scene node
  text_node_->createChildSceneNode()->attachObject(text);

  // Update the materials
  updatePointsSize();
  updatePointsColor();
  updateLinesColor();
  updateTextSize();
  updateRenderAsOverlay();
}

void ZoneOfInterestTool::removeDisplays() {
  // Remove the displays
  // Points
  {
    const auto objects = points_node_->getAttachedObjects();
    points_node_->removeAndDestroyAllChildren();

    // Delete the objects attached to this node
    for (const auto object : objects) {
      scene_manager_->destroyManualObject(object->getName());
    }
  }

  // Lines
  {
    const auto objects = lines_node_->getAttachedObjects();
    lines_node_->removeAndDestroyAllChildren();

    // Delete the objects attached to this node
    for (const auto object : objects) {
      scene_manager_->destroyManualObject(object->getName());
    }
  }

  // Text
  for (Ogre::Node* child : text_node_->getChildren()) {
    auto* child_scene = dynamic_cast<Ogre::SceneNode*>(child);
    const auto objects = child_scene->getAttachedObjects();
    child_scene->removeAndDestroyAllChildren();

    for (const auto object : objects) {
      delete object;
    }
  }
  text_node_->removeAndDestroyAllChildren();
}

int ZoneOfInterestTool::processMouseEvent(rviz_common::ViewportMouseEvent& event) {
  // Set the visibility of this node off so the selection manager won't choose a point on our cursor mesh in the point
  // and patch
  cursor_node_->setVisible(false);

  // Project the pixel onto the XY plane (z == 0) of the fixed frame, so points can be selected even in free space
  const std::pair<bool, Ogre::Vector3> plane_point = projection_finder_->getViewportPointProjectionOnXYPlane(
    event.panel->getRenderWindow(), event.x, event.y);
  const bool got_point = plane_point.first;
  const Ogre::Vector3 position = plane_point.second;

  // Revisualize the cursor node
  cursor_node_->setVisible(true);

  // Set the cursor
  if (got_point) {
    setCursor(hit_cursor_);

    // Update the cursor position
    cursor_node_->setPosition(position);
    cursor_node_->setOrientation(Ogre::Quaternion::IDENTITY);
  } else {
    setCursor(std_cursor_);
  }

  // Collect the point
  if (event.leftUp()) {
    if (got_point) {
      points_.back().push_back(position);
      updateVisual();
    }
  } else if (event.middleUp()) {
    // Clear the selection
    if (points_.back().size() > 1) {
      // Clear the points data
      points_.back().clear();

      // Clear the points visualization
      {
        auto* points = dynamic_cast<Ogre::ManualObject*>(points_node_->getAttachedObjects().back());
        if (points) {
          points->clear();
        }
      }

      // Clear the lines visualization
      {
        auto* lines = dynamic_cast<Ogre::ManualObject*>(lines_node_->getAttachedObjects().back());
        if (lines) {
          lines->clear();
        }
      }

      // Make the text object invisible
      {
        const auto* text_node = dynamic_cast<Ogre::SceneNode*>(text_node_->getChildren().back());
        if (text_node) {
          text_node->setVisible(false);
        }
      }
    }
  } else if (event.rightUp()) {
    if (!points_.back().empty()) {
      updateText();

      // The current polygon has just been finished - announce the whole selection
      publishPolygons();

      newPolygon();
    }
  }
  return rviz_common::Tool::Render;
}

int ZoneOfInterestTool::processKeyEvent(QKeyEvent* event, rviz_common::RenderPanel* /*panel*/) {
  switch (event->key()) {
    case Qt::Key_Delete:
    case Qt::Key_Backspace:
      points_.clear();
      removeDisplays();
      newPolygon();

      // The latched topic has to be kept in sync with the (now empty) selection
      publishPolygons();
    break;
    default:
    break;
  }

  return rviz_common::Tool::Render;
}

void ZoneOfInterestTool::updateTopicName() {
  const std::string topic_string = topic_property_->getTopicStd();

  if (topic_string.empty()) {
    polygons_publisher_.reset();
    RVIZ_COMMON_LOG_WARNING("Zone of interest tool: the polygons topic is empty, the selection is not published");
    return;
  }

  rclcpp::Node::SharedPtr node = context_->getRosNodeAbstraction().lock()->get_raw_node();

  polygons_publisher_ = node->create_publisher<polygon_msgs::msg::Polygon2DCollection>(topic_string, qos_profile_);

  // If we are re-configuring the topic or its QoS, make the current selection available on it right away
  if (!points_.empty()) {
    publishPolygons();
  }
}

void ZoneOfInterestTool::updatePointsColor() {
  return updateMaterialColor(points_material_, pt_color_property_->getColor());
}

void ZoneOfInterestTool::updateLinesColor() {
  return updateMaterialColor(lines_material_, line_color_property_->getColor());
}

void ZoneOfInterestTool::updatePointsSize() {
  points_material_->setPointSize(pt_size_property_->getFloat());
}

void ZoneOfInterestTool::updateTextSize() {
  const float height = text_size_property_->getFloat();
  for (Ogre::Node* child : text_node_->getChildren()) {
    auto child_scene = dynamic_cast<Ogre::SceneNode*>(child);
    if (child_scene) {
      for (const auto child_scene_object : child_scene->getAttachedObjects()) {
        auto* text = dynamic_cast<rviz_rendering::MovableText*>(child_scene_object);
        if (text) {
          text->setCharacterHeight(height);
        }
      }
    }
  }
}

void ZoneOfInterestTool::updateTextVisibility() {
  const bool text_visible = text_visibility_property_->getBool();
  for (Ogre::Node* child : text_node_->getChildren()) {
    const auto* child_scene = dynamic_cast<Ogre::SceneNode*>(child);
    if (child_scene) {
      child_scene->setVisible(text_visible);
    }
  }
  text_size_property_->setHidden(!text_visible);
}

void ZoneOfInterestTool::updateText() {
  const bool show = points_.back().size() > 1;
  if (!show) {
    return;
  }

  // Get the last child scene node from the top-level text scene node
  auto* last_child_scene = dynamic_cast<Ogre::SceneNode*>(text_node_->getChildren().back());
  if (!last_child_scene || last_child_scene->numAttachedObjects() != 1) {
    return;
  }

  // Get the first attached object of the last child scene object
  auto* text = dynamic_cast<rviz_rendering::MovableText*>(last_child_scene->getAttachedObject(0));
  if (text == nullptr) {
    return;
  }

  text->setVisible(text_visibility_property_->getBool());

  // Set the position of the text node as the center of the points
  Ogre::Vector3 text_pos(0.0, 0.0, 0.0);
  for (const Ogre::Vector3& point : points_.back()) {
    text_pos += point;
  }
  text_pos /= static_cast<Ogre::Real>(points_.back().size());

  last_child_scene->setPosition(text_pos);
}

void ZoneOfInterestTool::updateRenderAsOverlay() {
  const bool render_as_overlay = render_as_overlay_property_->getBool();
  Ogre::RenderQueueGroupID render_id =
    render_as_overlay ? Ogre::RenderQueueGroupID::RENDER_QUEUE_OVERLAY : Ogre::RenderQueueGroupID::RENDER_QUEUE_MAIN;

  // Lines
  lines_material_->setDepthCheckEnabled(!render_as_overlay);
  lines_material_->setDepthWriteEnabled(!render_as_overlay);
  for (const auto object : lines_node_->getAttachedObjects()) {
    const auto lines = dynamic_cast<Ogre::ManualObject*>(object);
    if (!lines) {
      continue;
    }
    lines->setRenderQueueGroup(render_id);
  }

  // Points
  points_material_->setDepthCheckEnabled(!render_as_overlay);
  points_material_->setDepthWriteEnabled(!render_as_overlay);
  for (const auto object : points_node_->getAttachedObjects()) {
    const auto points = dynamic_cast<Ogre::ManualObject*>(object);
    if (!points) {
      continue;
    }
    points->setRenderQueueGroup(render_id);
  }

  // Text
  for (Ogre::Node* child : text_node_->getChildren()) {
    const auto scene_node = dynamic_cast<Ogre::SceneNode*>(child);
    if (!scene_node) {
      continue;
    }

    for (const auto object : scene_node->getAttachedObjects()) {
      const auto text = dynamic_cast<rviz_rendering::MovableText*>(object);
      if (!text) {
        continue;
      }

      text->setRenderQueueGroup(render_id);
      text->getMaterial()->setDepthCheckEnabled(!render_as_overlay);
      text->getMaterial()->setDepthWriteEnabled(!render_as_overlay);
    }
  }
}

void ZoneOfInterestTool::updateToolVisualization() {
  // Remove and destroy the first tool visualization from the scene node
  scene_manager_->destroyMovableObject(cursor_object_);

  // Create the new tool visualization
  cursor_object_ = createToolVisualization();

  // Attach the new tool visualization to the scene node
  cursor_node_->attachObject(cursor_object_);
  cursor_node_->setVisible(false);
}

Ogre::MovableObject* ZoneOfInterestTool::createToolVisualization() {
  return scene_manager_->createManualObject();
}

void ZoneOfInterestTool::publishPolygons() {
  if (!polygons_publisher_) {
    return;
  }

  polygon_msgs::msg::Polygon2DCollection msg;
  msg.header.frame_id = context_->getFixedFrame().toStdString();
  msg.header.stamp = context_->getRosNodeAbstraction().lock()->get_raw_node()->get_clock()->now();

  msg.polygons.reserve(points_.size());
  for (const std::vector<Ogre::Vector3>& polygon_points : points_) {
    // Skip selections with fewer than 3 points
    if (polygon_points.size() < 3) {
      continue;
    }

    polygon_msgs::msg::Polygon2D polygon;
    polygon.points.reserve(polygon_points.size());
    for (const Ogre::Vector3& pt : polygon_points) {
      polygon_msgs::msg::Point2D point;
      point.x = pt.x;
      point.y = pt.y;
      polygon.points.push_back(point);
    }

    msg.polygons.push_back(polygon);
  }

  polygons_publisher_->publish(msg);
}

void ZoneOfInterestTool::updateVisual() {
  points_material_->setPointSize(pt_size_property_->getFloat());

  const auto points = dynamic_cast<Ogre::ManualObject*>(points_node_->getAttachedObjects().back());
  if (!points) {
    return;
  }

  const auto lines = dynamic_cast<Ogre::ManualObject*>(lines_node_->getAttachedObjects().back());
  if (!lines) {
    return;
  }

  // Add the points to the display
  points->clear();
  points->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_POINT_LIST);
  for (std::size_t i = 0; i < points_.back().size(); ++i) {
    points->position(points_.back()[i]);
  }
  points->end();
  // Set the custom material (Ignore this: class "Ogre::ManualObject" has no member "setMaterial")
  points->setMaterial(0, points_material_);

  // Add the polygon lines
  if (static_cast<int>(points_.back().size()) > 1) {
    lines->clear();
    lines->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);

    for (std::size_t i = 0; i < points_.back().size() - 1; ++i) {
      lines->position(points_.back().at(i));
      lines->position(points_.back().at(i + 1));
    }

    // Close the polygon
    if (static_cast<int>(points_.back().size()) > 2) {
      lines->position(points_.back().back());
      lines->position(points_.back().front());
    }

    lines->end();
    // Set the custom material (Ignore this: class "Ogre::ManualObject" has no member "setMaterial")
    lines->setMaterial(0, lines_material_);
  }
}

}  // namespace rviz_zone_of_interest_tool

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_zone_of_interest_tool::ZoneOfInterestTool, rviz_common::Tool)
