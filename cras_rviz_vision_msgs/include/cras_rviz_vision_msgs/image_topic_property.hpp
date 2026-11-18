// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Topic property offering both raw and compressed camera images.
 * \author Czech Technical University in Prague
 */

#pragma once

#include <string>

#include <QString>  // NOLINT: the Qt headers have to be included before the ROS ones

#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace cras_rviz_vision_msgs {

/**
 * \brief Topic property listing the topics with the camera images the detection previews are taken from.
 *
 * rviz_common::RosTopicProperty can offer only the topics of a single message type, but the images can arrive either as
 * sensor_msgs/msg/Image, or as sensor_msgs/msg/CompressedImage.
 */
class ImageTopicProperty : public rviz_common::properties::RosTopicProperty {
  Q_OBJECT

public:
  //! Message type of the raw camera images.
  static constexpr const char* const RAW_TYPE = "sensor_msgs/msg/Image";
  //! Message type of the compressed camera images.
  static constexpr const char* const COMPRESSED_TYPE = "sensor_msgs/msg/CompressedImage";

  explicit ImageTopicProperty(const QString& name = QString(), const QString& description = QString(),
      rviz_common::properties::Property* parent = nullptr, const char* changed_slot = nullptr);

  /**
   * \brief Gives the property access to the ROS node, which it needs to learn what topics exist and what they carry.
   * \note Until it is called, the property offers no topics and does not know their types.
   */
  void initialize(rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node);

  /**
   * \brief Message type of the currently selected topic.
   * \return The type, or an empty string if the topic is not published (its name can also be typed by hand).
   */
  std::string selectedMessageType() const;

  /**
   * \brief Whether the given topic carries compressed images.
   *
   * A topic which is not published at the moment is recognized by its name (the compressed image topics created by
   * image_transport end with \c /compressed), so that the subscription is set up correctly even before the publisher
   * comes up.
   *
   * \param[in] topic Name of the topic.
   * \param[in] messageType Its message type, as reported by selectedMessageType() (may be empty).
   */
  static bool compressed(const std::string& topic, const std::string& messageType);

protected Q_SLOTS:
  void fillTopicList() override;

private:
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rvizRosNode_;
};

}  // namespace cras_rviz_vision_msgs
