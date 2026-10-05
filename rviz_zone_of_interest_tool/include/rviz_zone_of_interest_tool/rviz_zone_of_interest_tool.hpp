// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Czech Technical University in Prague

#pragma once

#include <OgreMaterial.h>
#include <OgreVector.h>
#include <polygon_msgs/msg/polygon2_d_collection.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rviz_common/tool.hpp>

#include <memory>
#include <vector>

namespace rviz_rendering {

class MaterialManager;
class MovableText;
class ViewportProjectionFinder;

}  // namespace rviz_rendering

namespace rviz_common::properties {

class BoolProperty;
class ColorProperty;
class FloatProperty;
class QosProfileProperty;
class RosTopicProperty;

}

class OgreManualObject;

namespace rviz_zone_of_interest_tool {

/**
 * \brief An RViz tool for drawing polygonal zones of interest.
 *
 * Left clicking adds points to the polygon that is currently being drawn, middle clicking clears it and right
 * clicking finishes it and starts a new one. Delete or Backspace deletes all the drawn polygons. Every change to
 * the finished polygons is announced on a latched topic so that they can be used by other ROS nodes.
 */
class ZoneOfInterestTool : public rviz_common::Tool {
  Q_OBJECT

public:
  /**
   * \brief Create the tool and assign its keyboard shortcut.
   */
  ZoneOfInterestTool();

  /**
   * \brief Delete the visualizations and the materials created by this tool.
   */
  ~ZoneOfInterestTool() override;

  /**
   * \brief Show the tool cursor.
   */
  void activate() override;

  /**
   * \brief Hide the tool cursor and finish the capture of the polygon that is being drawn.
   */
  void deactivate() override;

  /**
   * \brief Start a new, empty polygon and create the visualizations belonging to it.
   */
  void newPolygon();

  /**
   * \brief Handle mouse motion and clicks in the 3D view.
   *
   * The clicked pixel is projected onto the XY plane of the fixed frame, so that points can be selected even in
   * free space. A left click adds the projected point to the polygon that is being drawn, a middle click clears
   * that polygon and a right click finishes it, publishes the selection and starts a new polygon.
   *
   * \param[in] event The mouse event to handle.
   * \return \c rviz_common::Tool::Render so that the 3D view is redrawn.
   */
  int processMouseEvent(rviz_common::ViewportMouseEvent& event) override;

  /**
   * \brief Handle keyboard input.
   *
   * Delete and Backspace delete all the polygons that have been drawn so far.
   *
   * \param[in] event The key event to handle.
   * \param[in] panel The render panel the key event came from (not used).
   * \return \c rviz_common::Tool::Render so that the 3D view is redrawn.
   */
  int processKeyEvent(QKeyEvent* event, rviz_common::RenderPanel* panel) override;

public Q_SLOTS:
  /**
   * \brief (Re-)create the polygons publisher according to the selected topic name and QoS profile.
   */
  void updateTopicName();

  /**
   * \brief Set the color of the polygon points according to the point color property.
   */
  void updatePointsColor();

  /**
   * \brief Set the size of the polygon points according to the point size property.
   */
  void updatePointsSize();

  /**
   * \brief Set the color of the polygon lines according to the line color property.
   */
  void updateLinesColor();

  /**
   * \brief Redraw the points and lines of the polygon that is currently being drawn.
   */
  void updateVisual();

  /**
   * \brief Show or hide the polygon captions according to the show text property.
   */
  void updateTextVisibility();

  /**
   * \brief Set the height of the polygon captions according to the text size property.
   */
  void updateTextSize();

  /**
   * \brief Move the visualizations to the overlay or main render queue according to the render as overlay property.
   *
   * In the overlay queue depth testing is disabled so that the polygons are visible on top of all other geometry.
   */
  void updateRenderAsOverlay();

  /**
   * \brief Recreate the visualization of the tool cursor.
   */
  void updateToolVisualization();

protected:
  /**
   * \brief Create the object visualizing the tool cursor.
   * \return The newly created cursor object.
   */
  virtual Ogre::MovableObject* createToolVisualization();

  /**
   * \brief Set all color components of the given material to the given color.
   * \param[in] material The material to modify.
   * \param[in] color The color to set.
   * \param[in] override_self_illumination Whether to also set the self-illumination so that the material is visible
   *                                       even when it is not lit.
   */
  static void updateMaterialColor(
      Ogre::MaterialPtr material, const QColor& color, bool override_self_illumination = true);

  /**
   * \brief Create the tool properties, the polygon visualizations and the cursors.
   */
  void onInitialize() override;

private:
  /**
   * \brief Show the caption of the polygon that is currently being drawn in its center.
   */
  void updateText();

  /**
   * \brief Delete all visualizations of all polygons (points, lines and captions).
   */
  void removeDisplays();

  /**
   * \brief Publish all finished polygons on the configured latched topic.
   */
  void publishPolygons();

  /**
   * \brief The QoS profile the polygons are published with unless the user changes it.
   * \return A transient-local profile keeping only the last published message.
   */
  static rclcpp::QoS defaultQosProfile();

  /** \brief The topic the finished polygons are published on. */
  rviz_common::properties::RosTopicProperty* topic_property_;

  /** \brief QoS subproperties (depth, history, reliability, durability) underneath \ref topic_property_. */
  rviz_common::properties::QosProfileProperty* qos_profile_property_ {nullptr};

  /** \brief QoS profile currently selected by \ref qos_profile_property_. */
  rclcpp::QoS qos_profile_;

  /** \brief Property selecting the color of the polygon points. */
  rviz_common::properties::ColorProperty* pt_color_property_;
  /** \brief Property selecting the color of the polygon lines. */
  rviz_common::properties::ColorProperty* line_color_property_;
  /** \brief Property selecting the size of the polygon points. */
  rviz_common::properties::FloatProperty* pt_size_property_;
  /** \brief Property toggling the visibility of the polygon captions. */
  rviz_common::properties::BoolProperty* text_visibility_property_;
  /** \brief Property selecting the height of the polygon captions. */
  rviz_common::properties::FloatProperty* text_size_property_;
  /** \brief Property toggling whether the visualization is rendered on top of all other geometry. */
  rviz_common::properties::BoolProperty* render_as_overlay_property_;

  /** \brief Projects the clicked pixel onto the XY plane of the fixed frame (free-space clicks). */
  std::shared_ptr<rviz_rendering::ViewportProjectionFinder> projection_finder_;

  /** \brief Latched publisher of all polygons the user has finished. */
  rclcpp::Publisher<polygon_msgs::msg::Polygon2DCollection>::SharedPtr polygons_publisher_;

  /**
   * \brief Polygon points in 3D space.
   *
   * The outer vector corresponds to individual polygons. The last polygon is the one that is currently being drawn
   * and may still be unfinished.
   */
  std::vector<std::vector<Ogre::Vector3>> points_;

  // Visualizations

  /** \brief Parent scene node of the point visualization of each polygon. */
  Ogre::SceneNode* points_node_ {nullptr};
  /** \brief Parent scene node of the line visualization of each polygon. */
  Ogre::SceneNode* lines_node_ {nullptr};
  /** \brief Parent scene node of the caption of each polygon. */
  Ogre::SceneNode* text_node_ {nullptr};
  /** \brief Material of the point visualization. */
  Ogre::MaterialPtr points_material_ {nullptr};
  /** \brief Material of the line visualization. */
  Ogre::MaterialPtr lines_material_ {nullptr};
  /** \brief Cursor shown when no point can be selected on the XY plane of the fixed frame. */
  QCursor std_cursor_;
  /** \brief Cursor shown when a point can be selected. */
  QCursor hit_cursor_;
  /** \brief Scene node holding the tool cursor. */
  Ogre::SceneNode* cursor_node_ {nullptr};
  /** \brief Object visualizing the tool cursor. */
  Ogre::MovableObject* cursor_object_ {nullptr};
};

}  // namespace rviz_zone_of_interest_tool
