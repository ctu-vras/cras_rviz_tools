// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Keep-out zones the detections are tested against.
 * \author Czech Technical University in Prague
 */

#pragma once

#include <string>
#include <utility>
#include <vector>

namespace cras_rviz_vision_msgs {

/**
 * \brief Keep-out zones the detections can be inside of.
 *
 * The same polygons which the \ref ZoneTool draws and which the detection recorder of the g4s_mission package
 * evaluates. Only the horizontal position matters to them, so a zone is a closed outline in a single horizontal frame.
 */
class KeepoutZones {
public:
  //! One zone: the corners of its closed outline, in the frame of all the zones.
  using Zone = std::vector<std::pair<double, double>>;

  KeepoutZones() = default;

  /**
   * \brief Replaces the zones.
   * \param[in] frame Frame the positions of the corners are in.
   * \param[in] zones The zones; the ones which do not enclose an area are dropped.
   * \return Whether the zones are something else than before. The zones are published repeatedly and a publisher which
   *   is not asked otherwise sends the same ones again and again, so the users of the zones have to know whether they
   *   have to evaluate them for their content again.
   */
  bool set(std::string frame, std::vector<Zone> zones);

  //! \brief Forgets the zones, so that no detection can be told to be in (or out of) one.
  void clear();

  //! \brief Frame the positions of the corners are in.
  const std::string& getFrame() const {
    return frame_;
  }

  //! \brief Whether there is no zone to test a position against.
  bool empty() const {
    return zones_.empty();
  }

  //! \brief How many zones there are.
  size_t size() const {
    return zones_.size();
  }

  /**
   * \brief Which zones a position lies in.
   * \param[in] x Horizontal position, in the frame of the zones.
   * \param[in] y Vertical position, in the frame of the zones.
   * \return Numbers of the zones the position lies in (in the order they were published in), empty when it lies in
   *   none of them.
   */
  std::vector<int> zonesAt(double x, double y) const;

  /**
   * \brief Whether a position lies inside a closed outline.
   * \param[in] zone Corners of the outline, at least three of them.
   * \param[in] x Horizontal position to test.
   * \param[in] y Vertical position to test.
   * \return Whether the position is inside the outline; a position on its border counts as being inside.
   */
  static bool inside(const Zone& zone, double x, double y);

private:
  std::string frame_;
  std::vector<Zone> zones_;
};

}  // namespace cras_rviz_vision_msgs
