// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Implementation of the camera image topic property.
 * \author Czech Technical University in Prague
 */

#include "cras_rviz_vision_msgs/image_topic_property.hpp"

#include <algorithm>
#include <string>
#include <vector>

namespace cras_rviz_vision_msgs {

namespace {

//! Suffix of the compressed image topics created by image_transport.
const char COMPRESSED_SUFFIX[] = "/compressed";
//! Its length.
constexpr size_t COMPRESSED_SUFFIX_LENGTH = sizeof(COMPRESSED_SUFFIX) - 1;

}  // namespace

ImageTopicProperty::ImageTopicProperty(const QString& name, const QString& description,
    rviz_common::properties::Property* parent, const char* changed_slot)
    : RosTopicProperty(name, QString(), RAW_TYPE, description, parent, changed_slot) {
}

void ImageTopicProperty::initialize(rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node) {
  RosTopicProperty::initialize(rviz_ros_node);
  rvizRosNode_ = rviz_ros_node;
}

void ImageTopicProperty::fillTopicList() {
  const auto current = getValue().toString();
  clearOptions();

  const auto node = rvizRosNode_.lock();
  if (node) {
    for (const auto& topic : node->get_topic_names_and_types()) {
      const auto& types = topic.second;
      if (std::find(types.begin(), types.end(), RAW_TYPE) != types.end() ||
          std::find(types.begin(), types.end(), COMPRESSED_TYPE) != types.end())
      {
        addOptionStd(topic.first);
      }
    }
  }

  if (!current.isEmpty()) {
    // The selected topic may be currently not published, or it may carry something else; offer it anyway, so that the
    // value in the panel does not silently change to another topic. Note that clearing the options has reset the value
    // as a side effect, so it has to be set back explicitly.
    if (!strings_.contains(current)) {
      addOption(current);
    }
    if (current != getValue().toString()) {
      setValue(current);
    }
  }
  sortOptions();
}

std::string ImageTopicProperty::selectedMessageType() const {
  const auto node = rvizRosNode_.lock();
  const auto topic = getTopic().toStdString();
  if (!node || topic.empty()) {
    return {};
  }
  const auto topics = node->get_topic_names_and_types();
  const auto found = topics.find(topic);
  if (found == topics.end()) {
    return {};  // the topic is not published, so nobody knows what it should carry
  }
  const auto& types = found->second;
  if (std::find(types.begin(), types.end(), RAW_TYPE) != types.end()) {
    return RAW_TYPE;
  }
  if (std::find(types.begin(), types.end(), COMPRESSED_TYPE) != types.end()) {
    return COMPRESSED_TYPE;
  }
  return types.empty() ? std::string() : types.front();
}

bool ImageTopicProperty::compressed(const std::string& topic, const std::string& messageType) {
  if (messageType == COMPRESSED_TYPE) {
    return true;
  }
  if (!messageType.empty()) {
    return false;
  }
  // The topic is not published (or its type cannot be learned), so guess from its name. image_transport creates the
  // compressed publisher under the same name plus the /compressed suffix, and /theora does the same.
  return topic.size() >= COMPRESSED_SUFFIX_LENGTH &&
  topic.compare(topic.size() - COMPRESSED_SUFFIX_LENGTH, COMPRESSED_SUFFIX_LENGTH, COMPRESSED_SUFFIX) == 0;
}

}  // namespace cras_rviz_vision_msgs
