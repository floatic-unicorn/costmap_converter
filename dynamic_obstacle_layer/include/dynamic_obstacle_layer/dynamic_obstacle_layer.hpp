#ifndef DYNAMIC_OBSTACLE_LAYER_HPP_
#define DYNAMIC_OBSTACLE_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "costmap_converter_msgs/msg/obstacle_array_msg.hpp"
#include "costmap_converter_msgs/msg/obstacle_msg.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <mutex>
namespace dynamic_obstacle_plugin
{

class DynamicObstacleLayer : public nav2_costmap_2d::CostmapLayer
{
public:
    DynamicObstacleLayer();
    /*LayeredCostmap: virtual method API for working costmap layer on plugin */
    virtual void onInitialize();
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double * min_y, double * max_x, double * max_y);
    virtual void updateCosts(nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j);
    virtual void reset() {return;}
    virtual void onFootprintChanged();
    virtual bool isClearable() {return false;}
    std::mutex message_mutex_;
    costmap_converter_msgs::msg::ObstacleArrayMsg obstacleList_;
private:
    rclcpp::Subscription<costmap_converter_msgs::msg::ObstacleArrayMsg>::SharedPtr obstacle_sub_;
    void obstacleCallback(const costmap_converter_msgs::msg::ObstacleArrayMsg::SharedPtr msg);
    nav2_costmap_2d::Costmap2D seen_;
    double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
    bool need_recalculation_;
    int GRADIENT_SIZE = 20;
    int GRADIENT_FACTOR = 10;
    double sigma = 1.0;
    double mu = 0.0;
};

} // namespace
#endif