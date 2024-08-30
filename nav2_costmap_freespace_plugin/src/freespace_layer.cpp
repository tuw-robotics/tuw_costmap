#include "nav2_costmap_freespace_plugin/freespace_layer.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include "nav2_util/validate_messages.hpp"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::FREE_SPACE;

namespace nav2_costmap_freespace_plugin
{

FreespaceLayer::FreespaceLayer()
{
}

// This method is called at the end of plugin initialization.
// It contains ROS parameter(s) declaration and initialization
// of need_recalculation_ variable.
void
FreespaceLayer::onInitialize()
{
  global_frame_ = layered_costmap_->getGlobalFrameID();

  getParameters();


  rclcpp::QoS map_qos(10);  // initialize to default
  if (map_subscribe_transient_local_) {
    map_qos.transient_local();
    map_qos.reliable();
    map_qos.keep_last(1);
  }

  RCLCPP_INFO(
    logger_,
    "FreespaceLayer: Subscribing to the map topic (%s) with %s durability",
    map_topic_.c_str(),
    map_subscribe_transient_local_ ? "transient local" : "volatile");

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  map_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic_, map_qos,
    std::bind(&FreespaceLayer::incomingMap, this, std::placeholders::_1));


  current_ = true;
}

// The method is called to ask the plugin: which area of costmap it needs to update.
// Inside this method window bounds are re-calculated if need_recalculation_ is true
// and updated independently on its value.
void
FreespaceLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x,
  double * min_y,
  double * max_x,
  double * max_y)
{
  if (!map_received_) {
    map_received_in_update_bounds_ = false;
    return;
  }
  map_received_in_update_bounds_ = true;

  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());

  // If there is a new available map, load it.
  if (map_buffer_) {
    processMap(*map_buffer_);
    map_buffer_ = nullptr;
  }

  useExtraBounds(min_x, min_y, max_x, max_y);

  double wx, wy;

  mapToWorld(x_, y_, wx, wy);
  *min_x = std::min(wx, *min_x);
  *min_y = std::min(wy, *min_y);

  mapToWorld(x_ + width_, y_ + height_, wx, wy);
  *max_x = std::max(wx, *max_x);
  *max_y = std::max(wy, *max_y);
}

// The method is called when footprint was changed.
// Here it just resets need_recalculation_ variable.
void
FreespaceLayer::onFootprintChanged()
{
  RCLCPP_DEBUG(rclcpp::get_logger(
      "nav2_costmap_2d"), "FreespaceLayer::onFootprintChanged(): num footprint points: %lu",
    layered_costmap_->getFootprint().size());
}

// The method is called when costmap recalculation is required.
// It updates the costmap within its window bounds.
// Inside this method the costmap gradient is generated and is writing directly
// to the resulting costmap master_grid without any merging with previous layers.

void
FreespaceLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  RCLCPP_DEBUG(logger_, "FreespaceLayer: updateCosts");
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (!enabled_) {
    return;
  }
  if (!map_received_in_update_bounds_) {
    static int count = 0;
    // throttle warning down to only 1/10 message rate
    if (++count == 10) {
      RCLCPP_DEBUG(logger_, "FreespaceLayer: Can't update static costmap layer, no map received");
      count = 0;
    }
    return;
  }

  // update lookup table (the function itself checks if an update is needed)
  updateLookupTable(master_grid);

  // only update the costs if the lookup table exists and has the expeted size
  if (lookup_table_.size() == (master_grid.getSizeInCellsX()*master_grid.getSizeInCellsY())){
    
    unsigned char * master_costmap = master_grid.getCharMap();
    for (int i = min_i; i < max_i; ++i) {
      for (int j = min_j; j < max_j; ++j) {
        unsigned int idx_master = master_grid.getIndex(i, j);
        unsigned int idx_costmap = lookup_table_[idx_master];
        if(idx_costmap != std::numeric_limits<unsigned int>::max())
        {
          unsigned char value = costmap_[idx_costmap];
          // only update free space values
          if (value == nav2_costmap_2d::FREE_SPACE) {
            master_costmap[idx_master] = value;
          }
        }
      }
    }
  }
  current_ = true;
}

void
FreespaceLayer::updateLookupTable(
  nav2_costmap_2d::Costmap2D & master_grid){
  static unsigned int size_x = 0;
  static unsigned int size_y = 0;

  if(lookup_table_update_rule_ == LookupTableUpdateRule::ALLWAYS){
    lookup_table_update_ = true;
  }

  if(size_x != master_grid.getSizeInCellsX()){
    lookup_table_update_ = true;
  }

  if(size_y != master_grid.getSizeInCellsY()){
    lookup_table_update_ = true;
  }

  if(lookup_table_update_ == false || map_received_ == false) {
    return;
  }

  RCLCPP_INFO(logger_, "FreespaceLayer: updateLookupTable");

  size_x = master_grid.getSizeInCellsX();
  size_y = master_grid.getSizeInCellsY();
  lookup_table_.resize(size_x*size_y);

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_->lookupTransform(
      map_frame_, global_frame_, tf2::TimePointZero,
      transform_tolerance_);
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "FreespaceLayer: %s", ex.what());
    return;
  }
  // Copy map data given proper transformations
  tf2::Transform tf2_transform;

  tf2::fromMsg(transform.transform, tf2_transform);

  // The master_grid is unlikely to have same coordinates as this layer
  unsigned int mx, my;
  double wx, wy;

  for (unsigned int i = 0; i < size_y; ++i) {
    for (unsigned int j = 0; j < size_x; ++j) {
      // Convert master_grid coordinates (i,j) into global_frame_(wx,wy) coordinates
      layered_costmap_->getCostmap()->mapToWorld(j, i, wx, wy);
      unsigned int idx = master_grid.getIndex(j, i);
      // Transform from global_frame_ to map_frame_
      tf2::Vector3 p(wx, wy, 0);
      p = tf2_transform * p;
      // Set master_grid with cell from map
      unsigned int value = std::numeric_limits<unsigned int>::max();
      if (worldToMap(p.x(), p.y(), mx, my)) {
        value = getIndex(mx, my);
      } 
      lookup_table_[idx] = value;
    }
  }
  lookup_table_update_ = false;
}

unsigned char
FreespaceLayer::interpretValue(unsigned char value)
{
  // check if the static value is above the unknown or lethal thresholds
  if (value == 0) {
    return FREE_SPACE;
  } 
  return NO_INFORMATION;
}

void
FreespaceLayer::processMap(const nav_msgs::msg::OccupancyGrid & map_to_process)
{
  static nav_msgs::msg::OccupancyGrid map;
  if (map == map_to_process){
    return;
  }
  map = map_to_process;
  unsigned int size_x = map.info.width;
  unsigned int size_y = map.info.height;

  RCLCPP_INFO(
    logger_,
    "FreespaceLayer: processMap a %d X %d map at %f m/pix", size_x, size_y,
    map.info.resolution);

  // resize costmap if size, resolution or origin do not match
  if (size_x_ != size_x || size_y_ != size_y ||  // NOLINT
    resolution_ != map.info.resolution ||
    origin_x_ != map.info.origin.position.x ||
    origin_y_ != map.info.origin.position.y)
  {
    // only update the size of the costmap stored locally in this layer
    RCLCPP_INFO(
      logger_,
      "FreespaceLayer: Resizing static layer to %d X %d at %f m/pix", size_x, size_y,
      map.info.resolution);
    resizeMap(
      size_x, size_y, map.info.resolution,
      map.info.origin.position.x, map.info.origin.position.y);
  }
  unsigned int index = 0;

  // we have a new map, update full size of map
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  // initialize the costmap with static data
  for (unsigned int i = 0; i < size_y; ++i) {
    for (unsigned int j = 0; j < size_x; ++j) {
      unsigned char value = map.data[index];
      costmap_[index] = interpretValue(value);
      ++index;
    }
  }
  map_frame_ = map.header.frame_id;

  x_ = y_ = 0;
  width_ = size_x_;
  height_ = size_y_;

  current_ = true;
  lookup_table_update_ = true;

}

void
FreespaceLayer::incomingMap(const nav_msgs::msg::OccupancyGrid::SharedPtr new_map)
{
    RCLCPP_INFO(logger_, "FreespaceLayer: Received map");
  if (!nav2_util::validateMsg(*new_map)) {
    RCLCPP_ERROR(logger_, "FreespaceLayer: Received map message is malformed. Rejecting.");
    return;
  }
  if (true || !map_received_) {
    processMap(*new_map);
    map_received_ = true;
    return;
  }
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> guard(*nav2_costmap_2d::Costmap2D::getMutex());
  map_buffer_ = new_map;
}

void
FreespaceLayer::incomingUpdate(map_msgs::msg::OccupancyGridUpdate::ConstSharedPtr update)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  (void) update;
}

void
FreespaceLayer::getParameters()
{

  double temp_tf_tol = 0.0;
  std::string tmp_lookup_table_update_rule;

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("map_topic", rclcpp::ParameterValue(""));
  declareParameter("subscribe_to_updates", rclcpp::ParameterValue(false));
  declareParameter("map_subscribe_transient_local", rclcpp::ParameterValue(true));
  declareParameter("lookup_table_update_rule", rclcpp::ParameterValue("allways"));


  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "subscribe_to_updates", subscribe_to_updates_);
  node->get_parameter(name_ + "." + "map_subscribe_transient_local", map_subscribe_transient_local_);
  std::string private_map_topic, global_map_topic;
  node->get_parameter(name_ + "." + "map_topic", private_map_topic);
  node->get_parameter("map_topic", global_map_topic);
  if (!private_map_topic.empty()) {
    map_topic_ = private_map_topic;
  } else {
    map_topic_ = global_map_topic;
  }

  node->get_parameter("transform_tolerance", temp_tf_tol);
  transform_tolerance_ = tf2::durationFromSec(temp_tf_tol);

  node->get_parameter(name_ + "." + "lookup_table_update_rule", tmp_lookup_table_update_rule);
  std::transform(tmp_lookup_table_update_rule.begin(), tmp_lookup_table_update_rule.end(), tmp_lookup_table_update_rule.begin(), ::tolower);
  if(tmp_lookup_table_update_rule== "once") {
    lookup_table_update_rule_ = LookupTableUpdateRule::ONCE;
  } else if (tmp_lookup_table_update_rule == "allways")  {
    lookup_table_update_rule_ = LookupTableUpdateRule::ALLWAYS;
  } else  {
    lookup_table_update_rule_ = LookupTableUpdateRule::ONCE;
  }
  RCLCPP_INFO( logger_,
               "FreespaceLayer: lookup_table_update_rule is %s", 
               (lookup_table_update_rule_==LookupTableUpdateRule::ONCE?"once":"allways"));

  map_received_ = false;
  map_received_in_update_bounds_ = false;
}


}  // namespace nav2_costmap_freespace_plugin

// This is the macro allowing a nav2_gradient_costmap_plugin::FreespaceLayer class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_costmap_freespace_plugin::FreespaceLayer, nav2_costmap_2d::Layer)
