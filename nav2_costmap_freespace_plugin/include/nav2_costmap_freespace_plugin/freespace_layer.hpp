/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  Copyright (c) 2020, Samsung R&D Institute Russia
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *         Alexey Merzlyakov
 *
 * Reference tutorial:
 * https://navigation.ros.org/tutorials/docs/writing_new_costmap2d_plugin.html
 *********************************************************************/
#ifndef FREESPACE_LAYER_HPP_
#define FREESPACE_LAYER_HPP_

#include "map_msgs/msg/occupancy_grid_update.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"

namespace nav2_costmap_freespace_plugin
{

class FreespaceLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  FreespaceLayer();

  virtual void onInitialize();
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y);
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);

  virtual void reset()
  {
    return;
  }

  virtual void onFootprintChanged();

  virtual bool isClearable() {return false;}

protected:
  /**
   * @brief Get parameters of layer
   */
  void getParameters();


  /**
   * @brief Process a new map coming from a topic
   */
  void processMap(const nav_msgs::msg::OccupancyGrid & new_map);

  /**
   * @brief Interpret the value in the static map given on the topic to
   * convert into costs for the costmap to utilize
   */
  unsigned char interpretValue(unsigned char value);

void updateWithFreeSpace( nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j);
  /**
   * @brief  Callback to update the costmap's map from the map_server
   * @param new_map The map to put into the costmap. The origin of the new
   * map along with its size will determine what parts of the costmap's
   * static map are overwritten.
   */
  void incomingMap(const nav_msgs::msg::OccupancyGrid::SharedPtr new_map);

  /**
   * @brief Callback to update the costmap's map from the map_server (or SLAM)
   * with an update in a particular area of the map
   */
  void incomingUpdate(map_msgs::msg::OccupancyGridUpdate::ConstSharedPtr update);
private:


  std::string global_frame_;  ///< @brief The global frame for the costmap
  std::string map_frame_;  /// @brief frame that map is located in

  bool has_updated_data_{false};

  unsigned int x_{0};
  unsigned int y_{0};
  unsigned int width_{0};
  unsigned int height_{0};


  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::SharedPtr map_update_sub_;

  // Parameters
  bool map_subscribe_transient_local_;
  bool subscribe_to_updates_;
  std::string map_topic_;
  bool track_unknown_space_;
  bool use_maximum_;
  unsigned char lethal_threshold_;
  unsigned char unknown_cost_value_;
  bool trinary_costmap_;
  bool map_received_{false};
  bool map_received_in_update_bounds_{false};
  tf2::Duration transform_tolerance_;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_buffer_;
  // Indicates that the entire gradient should be recalculated next time.
  bool need_recalculation_;

  // Size of gradient in cells
  int GRADIENT_SIZE = 20;
  // Step of increasing cost per one cell in gradient
  int GRADIENT_FACTOR = 10;
};

}  // namespace nav2_costmap_freespace_plugin

#endif  // FREESPACE_LAYER_HPP_
