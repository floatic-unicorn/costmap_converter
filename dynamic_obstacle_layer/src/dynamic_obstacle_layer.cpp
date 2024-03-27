#include "dynamic_obstacle_layer/dynamic_obstacle_layer.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace dynamic_obstacle_plugin
{
DynamicObstacleLayer::DynamicObstacleLayer()
: last_min_x_(std::numeric_limits<double>::lowest()),
  last_min_y_(std::numeric_limits<double>::lowest()),
  last_max_x_(std::numeric_limits<double>::max()),
  last_max_y_(std::numeric_limits<double>::max())
  {}

void DynamicObstacleLayer::onInitialize()
{
    auto node = node_.lock();
    if (!node) {
    throw std::runtime_error{"Failed to lock node"};
    }
    obstacle_sub_ = node->create_subscription<costmap_converter_msgs::msg::ObstacleArrayMsg>("/dynamic_obstacle", rclcpp::SensorDataQoS(),
    std::bind(&DynamicObstacleLayer::obstacleCallback, this, std::placeholders::_1));
    RCLCPP_INFO(node->get_logger(),
                "DynamicObstacleLayer: subscribed to "
                "topic %s",
                obstacle_sub_->get_topic_name());
    declareParameter("enabled", rclcpp::ParameterValue(true));
    node->get_parameter(name_+"."+"enabled",enabled_);

    need_recalculation_ = false;
    current_ = true;
}
void DynamicObstacleLayer::obstacleCallback(const costmap_converter_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
{
    // costmap_converter_msgs::msg::ObstacleArrayMsg obstacleList_;
    message_mutex_.lock();
    obstacleList_ = *msg;
    message_mutex_.unlock();
}
void DynamicObstacleLayer::updateBounds(
    double, double, double, double * min_x, double * min_y, double * max_x, double * max_y
)
{
    if(!enabled_){ return; }
    
//    if(need_recalculation_)
//    {
//         last_min_x_ = *min_x;
//         last_min_y_ = *min_y;
//         last_max_x_ = *max_x;
//         last_max_y_ = *max_y;
//         *min_x = std::numeric_limits<double>::lowest();
//         *min_y = std::numeric_limits<double>::lowest();
//         *max_x = std::numeric_limits<double>::max();
//         *max_y = std::numeric_limits<double>::max();
//         need_recalculation_=false;
//    }
//    else
//    {
//         double tmp_min_x = last_min_x_;
//         double tmp_min_y = last_min_y_;
//         double tmp_max_x = last_max_x_;
//         double tmp_max_y = last_max_y_;
//         last_min_x_ = *min_x;
//         last_min_y_ = *min_y;
//         last_max_x_ = *max_x;
//         last_max_y_ = *max_y;
//         *min_x = std::min(tmp_min_x, *min_x);
//         *min_y = std::min(tmp_min_y, *min_y);
//         *max_x = std::max(tmp_max_x, *max_x);
//         *max_y = std::max(tmp_max_y, *max_y);
//    }
    // unsigned int mapX, mapY, index;
    // int predStep = 20;
    // double dt = 0.1;
        *min_x = std::numeric_limits<double>::lowest();
        *min_y = std::numeric_limits<double>::lowest();
        *max_x = std::numeric_limits<double>::max();
        *max_y = std::numeric_limits<double>::max();
    // for(int i = 0; i < obstacleList_.obstacles.size(); i++)
    // {
    //     costmap_converter_msgs::msg::ObstacleMsg obstacle;
    //     obstacle = obstacleList_.obstacles[i];
    //     //obstacle.header = obstacleList_.header;
        
    //     master_grid.worldToMap(obstacle.position.x,obstacle.position.y,mapX,mapY);
    //     *min_x = std::min(*min_x, static_cast<double>(mapX));
    //     *min_y = std::min(*min_y, static_cast<double>(mapY));
    //     *max_x = std::max(*max_x, static_cast<double>(mapX));
    //     *max_y = std::max(*max_y, static_cast<double>(mapY));
    //     //index = master_grid.getIndex(mapX,mapY);
    //     //master_array[index] = 254;
    //     // for(int j = 0; j<predStep; j++)
    //     // {
    //     //     obstacle.position.x += obstacle.velocity.x * dt; 
    //     //     obstacle.position.y += obstacle.velocity.y * dt;
    //     //     //if(abs(obstacle.position.x) >= 2.5 || abs(obstacle.position.y) >= 2.5)
    //     //     //    break;
    //     //     nav2_costmap_2d::Costmap2D::worldToMap(obstacle.position.x,obstacle.position.y,mapX,mapY);
    //     //     *min_x = std::min(*min_x, static_cast<double>(mapX));
    //     //     *min_y = std::min(*min_y, static_cast<double>(mapY));
    //     //     *max_x = std::max(*max_x, static_cast<double>(mapX));
    //     //     *max_y = std::max(*max_y, static_cast<double>(mapY));
    //     //     //index = master_grid.getIndex(mapX,mapY);
    //     //     //master_array[index] = 254;
    //     // }
    //     for(int k = 0; k<obstacle.polygon.points.size(); k++)
    //     {
    //         obstacle.polygon.points[k].x +=obstacle.velocity.x * (predStep*dt);
    //         obstacle.polygon.points[k].y +=obstacle.velocity.y * (predStep*dt);
    //         master_grid.worldToMap(obstacle.polygon.points[k].x,obstacle.polygon.points[k].y,mapX,mapY);
    //         index = master_grid.getIndex(mapX,mapY);
    //         if(index>= sizeX*sizeY || index < 0)
    //             continue;
    //         *min_x = std::min(*min_x, static_cast<double>(mapX));
    //         *min_y = std::min(*min_y, static_cast<double>(mapY));
    //         *max_x = std::max(*max_x, static_cast<double>(mapX));
    //         *max_y = std::max(*max_y, static_cast<double>(mapY));
            
    //         //RCLCPP_INFO(rclcpp::get_logger("dynamic_obstacle_plugin"),"Pred (%.2f, %.2f) -> (%d,%d)",obstacle.position.x,obstacle.position.y,mapX,mapY); 
    //     }
    // }

    //RCLCPP_INFO(rclcpp::get_logger("dynamic_obstacle_plugin"),"UpdateBounds!! min (%.2f, %.2f) max (%.2f, %.2f)",min_x,min_y,max_x,max_y);
}
void DynamicObstacleLayer::onFootprintChanged()
{
    need_recalculation_ = true;

    RCLCPP_DEBUG(rclcpp::get_logger("nav2_costmap_2d"), "GradientLayer::onFootprintChanged(): num footprint points: %lu",layered_costmap_->getFootprint().size());

}
void DynamicObstacleLayer::updateCosts(nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
    if(!enabled_){ return; }
    //RCLCPP_INFO(rclcpp::get_logger("dynamic_obstacle_plugin"),"UpdateCosts1");
    //master_array - direct pointer to the resulting master_grid
    //master_grid - resulting costmap combined from all layers
    unsigned char * master_array = master_grid.getCharMap();
    unsigned int sizeX = master_grid.getSizeInCellsX(), sizeY = master_grid.getSizeInCellsY();
    //{min_i, min_j} - {max_i, max_j} are update-window coordinates.
    //These variables are used to update the costmap only within this window
    //avoiding the updates of whole area
    unsigned int mapX, mapY, index;
    int predStep = 30;
    double dt = 0.1;
    double originX = master_grid.getOriginX()+master_grid.getSizeInMetersX()/2;
    double originY = master_grid.getOriginY()+master_grid.getSizeInMetersY()/2;
    double sizeXmeter = master_grid.getSizeInMetersX();
    double sizeYmeter = master_grid.getSizeInMetersY();

    for(int i = 0; i < obstacleList_.obstacles.size(); i++)
    {
        costmap_converter_msgs::msg::ObstacleMsg obstacle;
        obstacle = obstacleList_.obstacles[i];
        double originToObsX = abs(obstacle.position.x - originX)+0.05; 
        double originToObsY = abs(obstacle.position.y - originY)+0.05; 
        //obstacle.header = obstacleList_.header;
        if(abs(obstacle.velocity.x) < 0.05 && abs(obstacle.velocity.y) < 0.05)
            continue;
        if(originToObsX > sizeXmeter/2 || originToObsY > sizeYmeter/2)
            continue;
        master_grid.worldToMap(obstacle.position.x,obstacle.position.y,mapX,mapY);
        index = master_grid.getIndex(mapX,mapY);
        if(index>= sizeX*sizeY)
            continue;
        //RCLCPP_INFO(rclcpp::get_logger("dynamic_obstacle_plugin"),"mapSize (%d,%d) Start (%d,%d)->index %d ",sizeX,sizeY,mapX,mapY,index);
        master_array[index] = 254;
        for(int j = 0; j<predStep; j++)
        {
            for(int k = 0; k<obstacle.polygon.points.size(); k++)
            {
                obstacle.polygon.points[k].x +=obstacle.velocity.x * dt;
                obstacle.polygon.points[k].y +=obstacle.velocity.y * dt;
                originToObsX = abs(obstacle.polygon.points[k].x - originX)+0.05; 
                originToObsY = abs(obstacle.polygon.points[k].y - originY)+0.05; 
                if(abs(originToObsX) > sizeXmeter/2 || abs(originToObsY) > sizeYmeter/2)
                    continue;
                master_grid.worldToMap(obstacle.polygon.points[k].x,obstacle.polygon.points[k].y,mapX,mapY);
                index = master_grid.getIndex(mapX,mapY);
                if(index>= sizeX*sizeY || index < 0)
                    continue;
                master_array[index] = 254;
                //RCLCPP_INFO(rclcpp::get_logger("dynamic_obstacle_plugin"),"Pred (%.2f, %.2f) -> (%d,%d)",obstacle.position.x,obstacle.position.y,mapX,mapY); 
            }
            //obstacle.position.x += obstacle.velocity.x * dt; 
            //obstacle.position.y += obstacle.velocity.y * dt;
            //if(abs(obstacle.position.x) >= 2.5 || abs(obstacle.position.y) >= 2.5)
            //    break;
            // nav2_costmap_2d::Costmap2D::worldToMap(obstacle.position.x,obstacle.position.y,mapX,mapY);
            // index = master_grid.getIndex(mapX,mapY);
            // master_array[index] = 254;
            // RCLCPP_INFO(rclcpp::get_logger("dynamic_obstacle_plugin"),"Pred (%.2f, %.2f) -> (%d,%d)",obstacle.position.x,obstacle.position.y,mapX,mapY);
        }

    }
    //RCLCPP_INFO(rclcpp::get_logger("dynamic_obstacle_plugin"),"UpdateCosts2");
    //Fixing window coordinates with map size if necessary.
    min_i = std::max(0, min_i);
    min_j = std::max(0, min_j);
    max_i = std::min(static_cast<int>(sizeX),max_i);
    max_j = std::min(static_cast<int>(sizeY),max_j);
    //RCLCPP_INFO(rclcpp::get_logger("dynamic_obstacle_plugin"),"UpdateCosts3");
    // int gradient_index; 
    // for(int j = min_j; j < max_j; j++)
    // {
    //     //Reset gradient_index each time when reaching the end of re-calculated window by OY axis
    //     gradient_index = 0;
    //     for ( int i = min_i; i< max_i; i++ )
    //     {
    //         int index = master_grid.getIndex(i, j);
    //         unsigned char cost = (LETHAL_OBSTACLE - gradient_index*GRADIENT_FACTOR)%255;
    //         if (gradient_index <= GRADIENT_SIZE)
    //         {
    //             gradient_index++;
    //         }
    //         else
    //         {
    //             gradient_index = 0;
    //         }
    //         master_array[index] = cost;
    //     }
    // }
}
} // namespace

// This is the macro allowing a nav2_gradient_costmap_plugin::GradientLayer class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dynamic_obstacle_plugin::DynamicObstacleLayer, nav2_costmap_2d::Layer)