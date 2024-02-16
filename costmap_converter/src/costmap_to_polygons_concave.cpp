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

    static int obsCount = 0;
    TrackerContainerPtr currentFrameTrackers(new std::vector<KalmanEigen>());
    vector<redefObs> redefinedObstacles;
    vector<redefObs> assignedObstacles;
    vector<vector<double>> distanceMatrix;
    vector<int> assignment;

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
    pair<double,double> obstaclePosition;
    pair<double,double> obstacleVelocity;
    
    int historyThreshold = 1;
    int obstaclesSize = polygons->size();
    //RCLCPP_INFO(getLogger(), "tracking start ");
    if(trackers_==nullptr)
    {
      RCLCPP_INFO(getLogger(), "Empty Tracker..");
        
      for(int i=0; i< obstaclesSize; i++)
      {
        double pSumX = 0;
        double pSumY = 0;
        size_t len = (*polygons)[i].points.size();

        for(int j=0; j<len; j++)
        {
  
          pSumX += (*polygons)[i].points[j].x;
          pSumY += (*polygons)[i].points[j].y;
        }
        double pMeanX = pSumX/len;
        double pMeanY = pSumY/len;
        obstaclePosition = make_pair(pMeanX,pMeanY);
        obsCount++;
        KalmanEigen tracker = KalmanEigen(obstaclePosition,obsCount);
        currentFrameTrackers->push_back(tracker);
        trackers_ = currentFrameTrackers;
      }
      RCLCPP_INFO(getLogger(), "Input %d obstacle to trackers",currentFrameTrackers->size());
    }
    else
    {
      int trackerSize = trackers_->size();
      //////////////////////////////////////////////////////////////////////
      ////////////////////////Redefined Obstacle////////////////////////////
      for(int i=0; i< obstaclesSize; i++)
      {
        double pSumX = 0;
        double pSumY = 0;
        size_t len = (*polygons)[i].points.size();
        //RCLCPP_INFO(getLogger(), "Polygon %dth PointNum %d",i,len);
        for(int j=0; j<len; j++)
        {
          
          pSumX += (*polygons)[i].points[j].x;
          pSumY += (*polygons)[i].points[j].y;
        }
        double pMeanX = pSumX/len;
        double pMeanY = pSumY/len;
        redefObs obs = {pMeanX, pMeanY};
        redefinedObstacles.push_back(obs);
      }

      //////////////////////////////////////////////////////////////////////
      ////////////////////////DistanceMatrix Initialize/////////////////////
      distanceMatrix.clear();
      distanceMatrix.resize(trackerSize,vector<double>(obstaclesSize,0));
      for(int i = 0; i<trackerSize; i++)
      {
        (*trackers_)[i].predict();
      }
      for (int i=0; i<redefinedObstacles.size(); i++)
      {
        double pMeanX = redefinedObstacles[i].x;
        double pMeanY = redefinedObstacles[i].y;
        RCLCPP_INFO(getLogger(),"Obstacle pos(%.2f,%.2f)",pMeanX,pMeanY);  
        //RCLCPP_INFO(getLogger(),"Shape %d, %d".distanceMatrix,distanceMatrix[0].size());
        for(int j = 0; j < trackers_->size(); j++)
        {
          pair<double, double> trackerState = (*trackers_)[j].getState(0);
          distanceMatrix[j][i] = sqrt((pMeanX-trackerState.first) * (pMeanX-trackerState.first) + (pMeanY-trackerState.second) * (pMeanY-trackerState.second)); 
          //RCLCPP_INFO(getLogger(),"trk %d, obs %d distance %f trkpos %f %f obspos %f %f",j,i,distanceMatrix[j][i],trackerState.first,trackerState.second,pMeanX,pMeanY);
        }
      }
      /////////////////////////////////////////////////////////////////////
      ///////////////////////Matching//////////////////////////////////////
      HungAlgo.Solve(distanceMatrix, assignment);
      // for(int i = trackers_->size()-1; i>-1; i--)
      // {
      //   RCLCPP_INFO(getLogger(),"Size %d id %d Assign %d -> %d :: distance %f",trackers_->size(),(*trackers_)[i].id, i,assignment[i], distanceMatrix[i][assignment[i]]);  
      // }
      for(int i = 0; i<redefinedObstacles.size(); i++)
      {
       
      }
      if(redefinedObstacles.size())
      {
        for(int i = trackers_->size()-1; i>-1; i--)
        {
          if(assignment[i] == -1)
          {
            (*trackers_)[i].unmatchedHistory++;
            if((*trackers_)[i].unmatchedHistory > historyThreshold)
            {
              RCLCPP_INFO(getLogger(),"%dth Tracker UnmatchedHistory %d, So erase it. Remain Size %d", (*trackers_)[i].id, (*trackers_)[i].unmatchedHistory, trackers_->size());
              trackers_->erase(trackers_->begin() + i);
            }
          }
          else
          {
            if(distanceMatrix[i][assignment[i]] < 0.5 && assignment[i] != -1)
            {
              assignedObstacles.push_back(redefinedObstacles[assignment[i]]);
              (*trackers_)[i].update(make_pair(redefinedObstacles[assignment[i]].x,redefinedObstacles[assignment[i]].y));
              obstaclePosition = (*trackers_)[i].getState(0);
              obstacleVelocity = (*trackers_)[i].getState(1);
              double velocity = sqrt(obstacleVelocity.first*obstacleVelocity.first+obstacleVelocity.second*obstacleVelocity.second);
              //if(velocity > 0.5)
              //RCLCPP_INFO(getLogger()," Assign %d -> %d :: distance %f",(*trackers_)[i].id, assignment[i], distanceMatrix[i][assignment[i]]);
              //RCLCPP_INFO(getLogger()," trk %d of %d UnmatchHistory %d, obs %d, distance %lf",(*trackers_)[i].id, trackers_->size(), (*trackers_)[i].unmatchedHistory, assignment[i], distanceMatrix[i][assignment[i]]);
              
              RCLCPP_INFO(getLogger(),"Tracker id:%d pos(%.2f,%.2f) vel %f",(*trackers_)[i].id,obstaclePosition.first,obstaclePosition.second,velocity);
              //redefinedObstacles.erase(redefinedObstacles.begin()+assignment[i]);
              //RCLCPP_INFO(getLogger(),"Assign %d -> %d :: distance %f",(*trackers_)[i].id, assignment[i], distanceMatrix[i][assignment[i]]); 
            }
            else
            {
              (*trackers_)[i].unmatchedHistory++;
              if((*trackers_)[i].unmatchedHistory > historyThreshold)
              {
                RCLCPP_INFO(getLogger(),"%dth Tracker UnmatchedHistory %d, So erase it. Remain Size %d", (*trackers_)[i].id, (*trackers_)[i].unmatchedHistory, trackers_->size());
              
                trackers_->erase(trackers_->begin() + i);
              }
            }
          }
        }
        for(auto obs1 : redefinedObstacles)
        {
          bool newsignal = true;
          for(auto obs2 : assignedObstacles)
          {
            if(obs1.x == obs2.x && obs1.y == obs2.y)
              newsignal=false; 
          }
          if(newsignal)
          {
            obstaclePosition = make_pair(obs1.x,obs1.y);
            obsCount++;
            KalmanEigen tracker = KalmanEigen(obstaclePosition,obsCount);
            trackers_->push_back(tracker);
            RCLCPP_INFO(getLogger(),"newDefinedTracker %dth %f %f",obsCount,obstaclePosition.first, obstaclePosition.second);
          }
        }
        // for(int i =0; i<assignment.size(); i++)
        // {
        //     if(distanceMatrix[i][assignment[i]]> 0.0000000 && distanceMatrix[i][assignment[i]] < 1.0)
        //     {
        //       currentFrameTrackers = trackers_;
        //       (*currentFrameTrackers)[i].predict(0.05);
        //       (*currentFrameTrackers)[i].update(make_pair(redefinedObstacles[assignment[i]].x,redefinedObstacles[assignment[i]].y));
        //       RCLCPP_INFO(getLogger(),"Assign %d to %d, Distance %f",i,assignment[i],distanceMatrix[i][assignment[i]]);
        //       trackers_ = currentFrameTrackers;
        //     }
            
        //     if(assignment[i] == -1)
        //     {
        //       (*trackers_).erase((*trackers_).begin()+i);
        //     }
            
        // }
      }
      else  
      {
        for(int i = trackers_->size()-1; i>-1; i--)
        {
          RCLCPP_INFO(getLogger(),"noObstacles::");
          (*trackers_)[i].unmatchedHistory++;
            if((*trackers_)[i].unmatchedHistory > historyThreshold){
              RCLCPP_INFO(getLogger(),"%dth Tracker UnmatchedHistory %d, So erase it. Remain Size %d", (*trackers_)[i].id, (*trackers_)[i].unmatchedHistory, trackers_->size());
              trackers_->erase(trackers_->begin() + i);
              }
        }
      }
      for(int i = 0; i<trackers_->size(); i++)
      {
        (*trackers_)[i].predict();
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


