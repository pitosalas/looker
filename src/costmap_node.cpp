#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <nav2_util/lifecycle_node.hpp>

class SimpleCostmapNode : public nav2_util::LifecycleNode
{
public:
  SimpleCostmapNode() : nav2_util::LifecycleNode("simple_costmap_node")
  {
    RCLCPP_INFO(get_logger(), "Creating SimpleCostmapNode");
  }

  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override
  {
    RCLCPP_INFO(get_logger(), "Configuring XX %s", state.label().c_str());
    
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("local_costmap");
    costmap_->configure();
    
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override
  {
    RCLCPP_INFO(get_logger(), "Activating %s", state.label().c_str());
    
    // Activate the costmap
    costmap_->activate();
    
    // Create a timer to periodically display costmap info
      timer_ = create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&SimpleCostmapNode::timer_callback, this));
    
    return nav2_util::CallbackReturn::SUCCESS;
  }

  private:

  void timer_callback()
    {
      auto costmap = costmap_->getCostmap();
      RCLCPP_INFO(
        get_logger(),
        "Costmap info - Size: %d x %d cells, Resolution: %.3f m/cell, Origin: (%.2f, %.2f)",
        costmap->getSizeInCellsX(),
        costmap->getSizeInCellsY(),
        costmap->getResolution(),
        costmap->getOriginX(),
        costmap->getOriginY());
    }
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleCostmapNode>();
  
  // Use the lifecycle manager to handle transitions
  // rclcpp_lifecycle::State state;
  node->configure();
  std::cout << "about to call activate" << std::endl;

  node->activate();
  std::cout << "just called activate"<< std::endl;
  
  rclcpp::spin(node->get_node_base_interface());
  
  // Clean up
  node->deactivate();
  node->cleanup();
  
  rclcpp::shutdown();
  return 0;
}