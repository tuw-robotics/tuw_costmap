#ifndef FREESPACE_LAYER_HPP_
#define FREESPACE_LAYER_HPP_

#include "map_msgs/msg/occupancy_grid_update.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"

namespace nav2_costmap_freespace_plugin
{
enum LookupTableUpdateRule{
  ONCE,
  ALLWAYS
};

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

  /**
   * @brief  Callback to update the costmap's map from the map_server
   * @param new_map The map to put into the costmap. The origin of the new
   * map along with its size will determine what parts of the costmap's
   * static map are overwritten.
   */
  void incomingMap(const nav_msgs::msg::OccupancyGrid::SharedPtr new_map);

  /**
   * @brief  Updates the lookup table which manages the master and local map relation
   * @param  master_grid 
   */
  void updateLookupTable(nav2_costmap_2d::Costmap2D & master_grid);

  /**
   * @brief Callback to update the costmap's map from the map_server (or SLAM)
   * with an update in a particular area of the map
   */
  void incomingUpdate(map_msgs::msg::OccupancyGridUpdate::ConstSharedPtr update);
private:


  std::string global_frame_;  ///< @brief The global frame for the costmap
  std::string map_frame_;  /// @brief frame that map is located in

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
  bool map_received_{false};
  bool map_received_in_update_bounds_{false};
  tf2::Duration transform_tolerance_;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_buffer_;

  // to update the master of with the current layer master x,y -> costmap idx 
  std::vector<unsigned int> lookup_table_;  
  // Rule how the lookup table shoud be updated 
  LookupTableUpdateRule lookup_table_update_rule_;
  bool lookup_table_update_{true}; // on true the lookup table will be updated before updateing the costs
};

}  // namespace nav2_costmap_freespace_plugin

#endif  // FREESPACE_LAYER_HPP_
