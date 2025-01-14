/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
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
 *   * Neither the name of the institute nor the names of its
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
 * Author: Christoph Rösmann, Otniel Rinaldo
 *********************************************************************/

#include <costmap_converter/costmap_to_polygons_concave.h>

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(costmap_converter::CostmapToPolygonsDBSConcaveHull, costmap_converter::BaseCostmapToPolygons)

namespace costmap_converter
{
    
CostmapToPolygonsDBSConcaveHull::CostmapToPolygonsDBSConcaveHull() : CostmapToPolygonsDBSMCCH()
{
//  dynamic_recfg_ = NULL;
}

CostmapToPolygonsDBSConcaveHull::~CostmapToPolygonsDBSConcaveHull() 
{
//  if (dynamic_recfg_ != NULL)
//    delete dynamic_recfg_;
}

void CostmapToPolygonsDBSConcaveHull::initialize(rclcpp::Node::SharedPtr nh)
{ 
  CostmapToPolygonsDBSMCCH::initialize(nh);
  concave_hull_depth_ = declareAndGetParam(nh, "concave_hull_depth", 2.0);
  RCLCPP_INFO(nh->get_logger(), "concave hull depth : %f", concave_hull_depth_);
}


void CostmapToPolygonsDBSConcaveHull::compute()
{
    std::vector< std::vector<KeyPoint> > clusters;
    dbScan(clusters);
    
    // Create new polygon container
    PolygonContainerPtr polygons(new std::vector<geometry_msgs::msg::Polygon>());
    
    
    // add convex hulls to polygon container
    for (size_t i = 1; i <clusters.size(); ++i) // skip first cluster, since it is just noise
    {
      polygons->push_back( geometry_msgs::msg::Polygon() );
      concaveHull(clusters[i], concave_hull_depth_, polygons->back() );
    }
    
    // add our non-cluster points to the polygon container (as single points)
    if (!clusters.empty())
    {
      for (size_t i=0; i < clusters.front().size(); ++i)
      {
        polygons->push_back( geometry_msgs::msg::Polygon() );
        convertPointToPolygon(clusters.front()[i], polygons->back());
      }
    }
    
    // replace shared polygon container
    updatePolygonContainer(polygons);
}


void CostmapToPolygonsDBSConcaveHull::concaveHull(std::vector<KeyPoint>& cluster, double depth, geometry_msgs::msg::Polygon& polygon)
{
    // start with convex hull
    convexHull2(cluster, polygon);

    std::vector<geometry_msgs::msg::Point32>& concave_list = polygon.points;

    for (int i = 0; i < (int)concave_list.size() - 1; ++i)
    {
      
        // find nearest inner point pk from line (vertex1 -> vertex2)
        const geometry_msgs::msg::Point32& vertex1 = concave_list[i];
        const geometry_msgs::msg::Point32& vertex2 = concave_list[i+1];

        bool found;
        size_t nearest_idx = findNearestInnerPoint(vertex1, vertex2, cluster, concave_list, &found);
        if (!found) 
          continue;  
        
        double line_length = norm2d(vertex1, vertex2);
                
        double dst1 = norm2d(cluster[nearest_idx], vertex1);
        double dst2 = norm2d(cluster[nearest_idx], vertex2);
        double dd = std::min(dst1, dst2);
        if (dd<1e-8)
          continue;

        if (line_length / dd > depth)
        {
            // Check that new candidate edge will not intersect existing edges.
            bool intersects = checkLineIntersection(concave_list, vertex1, vertex2, vertex1, cluster[nearest_idx]);
            intersects |= checkLineIntersection(concave_list, vertex1, vertex2, cluster[nearest_idx], vertex2);
            if (!intersects) 
            {
              geometry_msgs::msg::Point32 new_point;
              cluster[nearest_idx].toPointMsg(new_point);
              concave_list.insert(concave_list.begin() + i + 1, new_point);
              i--;
            }
        }
    }
}


void CostmapToPolygonsDBSConcaveHull::concaveHullClusterCut(std::vector<KeyPoint>& cluster, double depth, geometry_msgs::msg::Polygon& polygon)
{
    // start with convex hull
    convexHull2(cluster, polygon);

    std::vector<geometry_msgs::msg::Point32>& concave_list = polygon.points;
    
    // get line length
    double mean_length = 0;
    for (int i = 0; i < (int)concave_list.size() - 1; ++i)
    {
      mean_length += norm2d(concave_list[i],concave_list[i+1]);
    }
    mean_length /= double(concave_list.size());

    for (int i = 0; i < (int)concave_list.size() - 1; ++i)
    {
      
        // find nearest inner point pk from line (vertex1 -> vertex2)
        const geometry_msgs::msg::Point32& vertex1 = concave_list[i];
        const geometry_msgs::msg::Point32& vertex2 = concave_list[i+1];

        double line_length = norm2d(vertex1, vertex2);
        
        bool found;
        size_t nearest_idx = findNearestInnerPoint(vertex1, vertex2, cluster, concave_list, &found);
        if (!found) 
        {
          continue;  
        }
        
                
        double dst1 = norm2d(cluster[nearest_idx], vertex1);
        double dst2 = norm2d(cluster[nearest_idx], vertex2);
        double dd = std::min(dst1, dst2);
        if (dd<1e-8)
          continue;

        if (line_length / dd > depth)
        {
            // Check that new candidate edge will not intersect existing edges.
            bool intersects = checkLineIntersection(concave_list, vertex1, vertex2, vertex1, cluster[nearest_idx]);
            intersects |= checkLineIntersection(concave_list, vertex1, vertex2, cluster[nearest_idx], vertex2);
            if (!intersects) 
            {
              geometry_msgs::msg::Point32 new_point;
              cluster[nearest_idx].toPointMsg(new_point);
              concave_list.insert(concave_list.begin() + i + 1, new_point);
              i--;
            }
        }
    }
}




//void CostmapToPolygonsDBSConcaveHull::reconfigureCB(CostmapToPolygonsDBSConcaveHullConfig& config, uint32_t level)
//{
//    boost::mutex::scoped_lock lock(parameter_mutex_);
//    parameter_buffered_.max_distance_ = config.cluster_max_distance;
//    parameter_buffered_.min_pts_ = config.cluster_min_pts;
//    parameter_buffered_.max_pts_ = config.cluster_max_pts;
//    parameter_buffered_.min_keypoint_separation_ = config.cluster_min_pts;
//    concave_hull_depth_ = config.concave_hull_depth;
//}

}//end namespace costmap_converter


