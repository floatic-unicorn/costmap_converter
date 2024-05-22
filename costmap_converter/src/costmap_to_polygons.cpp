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

#include <costmap_converter/costmap_to_polygons.h>
#include <costmap_converter/misc.h>
#include <costmap_converter/utills.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <boost/math/distributions/skew_normal.hpp>
#include <random>
PLUGINLIB_EXPORT_CLASS(costmap_converter::CostmapToPolygonsDBSMCCH, costmap_converter::BaseCostmapToPolygons)

namespace
{

/**
 * @brief Douglas-Peucker Algorithm for fitting lines into ordered set of points
 *
 * Douglas-Peucker Algorithm, see https://en.wikipedia.org/wiki/Ramer%E2%80%93Douglas%E2%80%93Peucker_algorithm
 *
 * @param begin iterator pointing to the begin of the range of points
 * @param end interator pointing to the end of the range of points
 * @param epsilon distance criteria for removing points if it is closer to the line segment than this
 * @param result the simplified polygon
 */
std::vector<geometry_msgs::msg::Point32> douglasPeucker(std::vector<geometry_msgs::msg::Point32>::iterator begin,
  std::vector<geometry_msgs::msg::Point32>::iterator end, double epsilon)
{
  if (std::distance(begin, end) <= 2)
  {
    return std::vector<geometry_msgs::msg::Point32>(begin, end);
  }

  // Find the point with the maximum distance from the line [begin, end)
  double dmax = std::numeric_limits<double>::lowest();
  std::vector<geometry_msgs::msg::Point32>::iterator max_dist_it;
  std::vector<geometry_msgs::msg::Point32>::iterator last = std::prev(end);
  
  for (auto it = std::next(begin); it != last; ++it)
  {
    double d = costmap_converter::computeSquaredDistanceToLineSegment(*it, *begin, *last);
    if (d > dmax)
    {
      max_dist_it = it;
      dmax = d;
    }
  }

  if (dmax < epsilon * epsilon)
  { // termination criterion reached, line is good enough
    std::vector<geometry_msgs::msg::Point32> result;
    result.push_back(*begin);
    result.push_back(*last);
    return result;
  }

  // Recursive calls for the two splitted parts
  auto firstLineSimplified = douglasPeucker(begin, std::next(max_dist_it), epsilon);
  auto secondLineSimplified = douglasPeucker(max_dist_it, end, epsilon);

  // Combine the two lines into one line and return the merged line.
  // Note that we have to skip the first point of the second line, as it is duplicated above.
  firstLineSimplified.insert(firstLineSimplified.end(),
    std::make_move_iterator(std::next(secondLineSimplified.begin())),
    std::make_move_iterator(secondLineSimplified.end()));
  return firstLineSimplified;
}

} // end namespace

namespace costmap_converter
{


//std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros;

CostmapToPolygonsDBSMCCH::CostmapToPolygonsDBSMCCH() : BaseCostmapToPolygons()
{
  costmap_ = NULL;
  // dynamic_recfg_ = NULL;
  neighbor_size_x_ = neighbor_size_y_ = -1;
  offset_x_ = offset_y_ = 0.;
}

CostmapToPolygonsDBSMCCH::~CostmapToPolygonsDBSMCCH()
{
//  if (dynamic_recfg_ != NULL)
//    delete dynamic_recfg_;
}

void CostmapToPolygonsDBSMCCH::initialize(rclcpp::Node::SharedPtr nh)
{
  BaseCostmapToPolygons::initialize(nh);

  costmap_ = NULL;
  RCLCPP_INFO(nh->get_logger(), "Dynamic Obstacle Tracker");
  
  parameter_.max_distance_ = declareAndGetParam(nh, "cluster_max_distance", 0.5);
  RCLCPP_INFO(nh->get_logger(), "cluster max distance : %f", parameter_.max_distance_);
  
  parameter_.min_pts_ = declareAndGetParam(nh, "cluster_min_pts", 4);
  RCLCPP_INFO(nh->get_logger(), "cluster min pts : %d", parameter_.min_pts_);
  
  parameter_.max_pts_ = declareAndGetParam(nh, "cluster_max_pts", 80);
  RCLCPP_INFO(nh->get_logger(), "cluster max pts : %d", parameter_.max_pts_);
  
  parameter_.min_keypoint_separation_ = declareAndGetParam(nh, "convex_hull_min_pt_separation", 0.1);
  RCLCPP_INFO(nh->get_logger(), "convex hull min pt separation : %f", parameter_.min_keypoint_separation_);

  parameter_buffered_ = parameter_;
  //costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("converter_costmap");
  //rclcpp_lifecycle::State state;
  //costmap_ros->on_configure(state);
  //costmap_ros->on_activate(state);
// setup dynamic reconfigure
//    dynamic_recfg_ = new dynamic_reconfigure::Server<CostmapToPolygonsDBSMCCHConfig>(nh);
//    dynamic_reconfigure::Server<CostmapToPolygonsDBSMCCHConfig>::CallbackType cb = boost::bind(&CostmapToPolygonsDBSMCCH::reconfigureCB, this, _1, _2);
//    dynamic_recfg_->setCallback(cb);
}


void CostmapToPolygonsDBSMCCH::compute()
{
    std::lock_guard<std::recursive_mutex> lock_guard(*costmap_->getMutex());
    int cells_x = int(costmap_->getSizeInMetersX() / parameter_.max_distance_) + 1;
    int cells_y = int(costmap_->getSizeInMetersY() / parameter_.max_distance_) + 1;
    if(cells_x != neighbor_size_x_ || cells_y != neighbor_size_y_){
      RCLCPP_INFO(getLogger(), " update costmap ");
      updateCostmap2D();
    }
    //updateCostmap2D();
    //RCLCPP_INFO(getLogger(), "1");
    std::vector< std::vector<KeyPoint> > clusters;
    
    dbScan(clusters);
    
    //RCLCPP_INFO(getLogger(), "2");
    static int obsCount = 0;
    //TrackerContainerPtr currentFrameTrackers = std::make_shared<std::vector<KalmanEigen>>();
    std::vector<KalmanEigen> currentFrameTrackers;
    vector<redefObs> redefinedObstacles;
    vector<redefObs> assignedObstacles;
    vector<vector<double>> distanceMatrix;
    vector<int> assignment;
    // Create new polygon container
    
    //RCLCPP_INFO(getLogger(), "3");
    PolygonContainerPtr polygons(new std::vector<geometry_msgs::msg::Polygon>());
    std::vector<geometry_msgs::msg::Polygon> cluster2;
    
    //RCLCPP_INFO(getLogger(), "4");

    // add convex hulls to polygon container
    // for (std::size_t i = 1; i <clusters.size(); ++i) // skip first cluster, since it is just noise
    // {
    //   polygons->push_back( geometry_msgs::msg::Polygon() );
    //   convexHull2(clusters[i], polygons->back() );
    // }

    // add our non-cluster points to the polygon container (as single points)
    // if (!clusters.empty())
    // {
    //   for (std::size_t i=0; i < clusters.front().size(); ++i)
    //   {
    //     polygons->push_back( geometry_msgs::msg::Polygon() );
    //     convertPointToPolygon(clusters.front()[i], polygons->back());
    //   }
    // }
    
    //polygons -> currentFrameDetectionObstacles
    pair<double,double> obstaclePosition;
    pair<double,double> obstacleVelocity;
    
    unsigned int historyThreshold = 1;
    int obstaclesSize = clusters.size()-1;
    int trackerSize = trackers_->size();
    //RCLCPP_INFO(getLogger(), "tracking start ");

    //////////////////////////////////////////////////////////////////////
    ////////////////////////Redefined Obstacle////////////////////////////
    for(int i=0; i< obstaclesSize; i++)
    {
      double pSumX = 0;
      double pSumY = 0;
      size_t len = clusters[i+1].size();
      //RCLCPP_INFO(getLogger(), "clusters size %d points %d",clusters.size(),clusters[i].size());
      // for(int j=0; j<len; j++)
      // {     
      //   pSumX += (*polygons)[i].points[j].x;
      //   pSumY += (*polygons)[i].points[j].y;
      //   //RCLCPP_INFO(getLogger(), "polygon(%d) %.2f %.2f",i,(*polygons)[i].points[j].x,(*polygons)[i].points[j].y);
      // }
      geometry_msgs::msg::Polygon cluster; 
      for(int j=0; j<len; j++)
      {
        cluster.points.push_back(geometry_msgs::msg::Point32());
        cluster.points.back().x = clusters[i+1][j].x;
        cluster.points.back().y = clusters[i+1][j].y;
        pSumX += clusters[i+1][j].x;
        pSumY += clusters[i+1][j].y;
        //RCLCPP_INFO(getLogger(), "cluster(%d) %.2f %.2f",i,cluster.points.back().x,cluster.points.back().y);
      }
      RCLCPP_INFO(getLogger(), "cluster size %d",cluster.points.size());
      cluster2.push_back(cluster);
      
      
      //double pMeanX = pSumX/len;
      //double pMeanY = pSumY/len;
      redefObs obs = {pSumX/len, pSumY/len, i};
      redefinedObstacles.push_back(obs);
    }
    
    
    if(!trackerSize && obstaclesSize)
    {
      RCLCPP_INFO(getLogger(), "Num of Tracking Objects = %d",trackerSize);
      for(auto obs1 : redefinedObstacles)
      {
        obstaclePosition = make_pair(obs1.x,obs1.y);
        obsCount++;
        KalmanEigen tracker = KalmanEigen(obstaclePosition,obsCount,obs1.id,cluster2[obs1.id]);
        trackers_->push_back(tracker);
        //RCLCPP_INFO(getLogger(),"newDefinedTracker %dth %f %f",obsCount,obstaclePosition.first, obstaclePosition.second);
      }
      if(trackerSize > 0)
        RCLCPP_INFO(getLogger(), "Input %d obstacle to trackers",trackerSize);
    }
    else if(trackerSize && obstaclesSize)
    {
      //////////////////////////////////////////////////////////////////////
      ////////////////////////DistanceMatrix Initialize/////////////////////
        for(int i = 0; i<trackerSize; i++)
        {
          (*trackers_)[i].predict();
        }
      //if(redefinedObstacles.size())
      //{
        distanceMatrix.clear();
        distanceMatrix.resize(trackerSize,vector<double>(obstaclesSize,0));
        //distanceMatrix.resize(obstaclesSize,vector<double>(trackerSize,0));
        for (int i=0; i<redefinedObstacles.size(); i++)
        {
          double pMeanX = redefinedObstacles[i].x;
          double pMeanY = redefinedObstacles[i].y;
          //RCLCPP_INFO(getLogger(),"Obstacle pos(%.2f,%.2f)",pMeanX,pMeanY);  
          //RCLCPP_INFO(getLogger(),"Shape %d, %d".distanceMatrix,distanceMatrix[0].size());
          for(int j = 0; j < trackerSize; j++)
          {
            pair<double, double> trackerState = (*trackers_)[j].getState(0);
            distanceMatrix[j][i] = sqrt((pMeanX-trackerState.first) * (pMeanX-trackerState.first) + (pMeanY-trackerState.second) * (pMeanY-trackerState.second)); 
            //RCLCPP_INFO(getLogger(),"trk %d, obs %d distance %f trkpos %f %f obspos %f %f",j,i,distanceMatrix[j][i],trackerState.first,trackerState.second,pMeanX,pMeanY);
          }
        }
      /////////////////////////////////////////////////////////////////////
      ///////////////////////Matching//////////////////////////////////////
        HungAlgo.Solve(distanceMatrix, assignment);
        for(int i = trackers_->size()-1; i>-1; i--)
        {
          if(assignment[i] == -1)
          {
            (*trackers_)[i].unmatchedHistory++;
            (*trackers_)[i].matchedHistory = 0;
            if((*trackers_)[i].unmatchedHistory > historyThreshold)
            {
              RCLCPP_INFO(getLogger(),"%dth Tracker UnmatchedHistory %d, So erase it. Remain Size %d", (*trackers_)[i].id, (*trackers_)[i].unmatchedHistory, trackers_->size());
              trackers_->erase(trackers_->begin() + i);
            }
          }
          else
          {
            //RCLCPP_INFO(getLogger(), "9");
            if(distanceMatrix[i][assignment[i]] < 0.5 && assignment[i] != -1)
            {
              
              assignedObstacles.push_back(redefinedObstacles[assignment[i]]);
              (*trackers_)[i].update(make_pair(redefinedObstacles[assignment[i]].x,redefinedObstacles[assignment[i]].y));
              (*trackers_)[i].obstacleId = redefinedObstacles[assignment[i]].id;
              (*trackers_)[i].pointList = cluster2[assignment[i]];
              obstaclePosition = (*trackers_)[i].getState(0);
              obstacleVelocity = (*trackers_)[i].getState(1);
              //double velocity = sqrt(obstacleVelocity.first*obstacleVelocity.first+obstacleVelocity.second*obstacleVelocity.second);
              //if(velocity > 0.5)
              //RCLCPP_INFO(getLogger()," Assign %d -> %d :: distance %f",(*trackers_)[i].id, assignment[i], distanceMatrix[i][assignment[i]]);
              //RCLCPP_INFO(getLogger()," trk %d of %d UnmatchHistory %d, obs %d, distance %lf",(*trackers_)[i].id, trackers_->size(), (*trackers_)[i].unmatchedHistory, assignment[i], distanceMatrix[i][assignment[i]]);
              //if(velocity >0.1)
              RCLCPP_INFO(getLogger(),"(%d)Tracker pos(%.2f,%.2f) vel(%.2f,%.2f)",(*trackers_)[i].id,obstaclePosition.first,obstaclePosition.second,obstacleVelocity.first,obstacleVelocity.second);
              //redefinedObstacles.erase(redefinedObstacles.begin()+assignment[i]);
              //RCLCPP_INFO(getLogger(),"Assign %d -> %d :: distance %f",(*trackers_)[i].id, assignment[i], distanceMatrix[i][assignment[i]]); 
            }
            else
            {
              (*trackers_)[i].matchedHistory = 0;
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
            KalmanEigen tracker = KalmanEigen(obstaclePosition,obsCount,obs1.id,cluster2[obs1.id]);
            trackers_->push_back(tracker);
            RCLCPP_INFO(getLogger(),"newDefinedTracker %dth %f %f",obsCount,obstaclePosition.first, obstaclePosition.second);
          }
        }
    }
    else if(trackerSize && !obstaclesSize)
    {
      for(int i = trackers_->size()-1; i>-1; i--)
        {
          (*trackers_)[i].unmatchedHistory++;
          (*trackers_)[i].matchedHistory = 0;
          if((*trackers_)[i].unmatchedHistory > historyThreshold)
          {
            RCLCPP_INFO(getLogger(),"%dth Tracker UnmatchedHistory %d, So erase it. Remain Size %d", (*trackers_)[i].id, (*trackers_)[i].unmatchedHistory, trackers_->size());
            trackers_->erase(trackers_->begin() + i);
          }
        }

    }
    
    // for(int i = 0; i<trackerSize; i++)
    // {
    //   (*trackers_)[i].predict();
    // }
    /////////////////////////////////////////////
    //////////To Filter unknown static obstacle//
    /////////////////////////////////////////////
    int mapSizeX = costmap_->getSizeInCellsX();
    int mapSizeY = costmap_->getSizeInCellsY();
    cv::Mat mapOfDynamicObstacle = cv::Mat::zeros(mapSizeX, mapSizeY, CV_8UC1);
    //RCLCPP_INFO(getLogger(),"Costmap Size (%d,%d)",costmap_->getSizeInCellsX(),costmap_->getSizeInCellsY());
    vector<int> obstacleId;
    vector<double> sampleList;
    trackerSize = trackers_->size();
    // for(int i = 0; i<trackers_->size(); i++)
    // {
    //   // if(!(*trackers_)[i].matchedHistory)
    //   //   continue;
    //   //RCLCPP_INFO(getLogger(), "CheckPoint..");
    //   obstacleVelocity = (*trackers_)[i].getState(1);
    //   //RCLCPP_INFO(getLogger(), "CheckPoint..0");
    //   if(abs(obstacleVelocity.first) < 0.05 && abs(obstacleVelocity.second) < 0.05)
    //   {
    //     //Add unknown static obstacle to ErasingObstacleList
    //     //trackers_->erase(trackers_->begin()+i);
    //     //obstacleId.push_back((*trackers_)[i].obsid);
    //   }
    //   else
    //   {
    //     int sampleTime = 500;
    //     //RCLCPP_INFO(getLogger(), "CheckPoint..1");
    //     pair<double, double> positionOfDynamicObstacle = (*trackers_)[i].getState(0); //0:get Position 1: get Velocity
    //     pair<double, double> velocityOfDynamicObstacle = (*trackers_)[i].getState(1);
    //     //RCLCPP_INFO(getLogger(), "CheckPoint..2");
    //     double estimatedYaw = 0.0;
    //     //if(velocityOfDynamicObstacle.first > 0)
    //     estimatedYaw = atan2(velocityOfDynamicObstacle.second, velocityOfDynamicObstacle.first)-(M_PI/2);
    //     unsigned int positionMx,positionMy;
    //     double resolution = 0.01;
    //     int rayBound = round((1.0)/resolution);
    //     double velocity = sqrt(velocityOfDynamicObstacle.first*velocityOfDynamicObstacle.first+velocityOfDynamicObstacle.second*velocityOfDynamicObstacle.second);
    //     //double rightBound = 0.3;
    //     // double frontBound = sqrt(-log(0.5/rightBound));
    //     // double rearBound = sqrt(-log(0.5/rightBound));
    //     //double variableBound = rayBound * resolution;
    //     //double functionBound = sqrt( -log( rayBound*resolution ) );
    //     for(int rayStep = 5; rayStep<rayBound; rayStep++)
    //     {
    //       double functionBound = sqrt( -3*velocity * log( 2*(rayStep*resolution) ) );
    //       double rearBound = sqrt( -0.1 * log( 2*(rayStep*resolution) ) );
    //       int forBound = round(functionBound/resolution);
    //         for(int y = 0; y < forBound; y++)
    //         {
    //           //double rayX = rayStep * resolution * cos(angleStep/180.0*M_PI);
    //           //double rayY = sqrt( -log( rayX ) );
    //           unsigned int gridX, gridY; 
    //           double frontLeftRayX =  rayStep * resolution;
    //           double frontLeftRayY = y * resolution;
    //           double frontRightRayX = - rayStep * resolution;
    //           double frontRightRayY = y * resolution;
    //           //RCLCPP_INFO(getLogger(), "CheckPoint..3");
    //           double globalX1 = positionOfDynamicObstacle.first + frontLeftRayX * cos(estimatedYaw) - frontLeftRayY * sin(estimatedYaw);
    //           double globalY1 = positionOfDynamicObstacle.second + frontLeftRayX * sin(estimatedYaw) + frontLeftRayY * cos(estimatedYaw);
    //           double globalX2 = positionOfDynamicObstacle.first + frontRightRayX * cos(estimatedYaw) - frontRightRayY * sin(estimatedYaw);
    //           double globalY2 = positionOfDynamicObstacle.second + frontRightRayX * sin(estimatedYaw) + frontRightRayY * cos(estimatedYaw);
    //           //RCLCPP_INFO(getLogger(), "CheckPoint..4 p1(%.2f, %.2f) p2(%.2f, %.2f)",globalX1,globalY1,globalX2,globalY2);
    //           // double globalX1 = positionOfDynamicObstacle.first + frontLeftRayX;
    //           // double globalY1 = positionOfDynamicObstacle.second + frontLeftRayY;
    //           // double globalX2 = positionOfDynamicObstacle.first + frontRightRayX;
    //           // double globalY2 = positionOfDynamicObstacle.second + frontRightRayY;
    //           if(abs(globalX1) < 2.5 && abs(globalY1) < 2.5)
    //           {
    //             //RCLCPP_INFO(getLogger(), "CheckPoint..if");
    //             //gridX = (globalX1 * cos (-M_PI/2) + globalY1 * sin(-M_PI/2))/resolution + 49;
    //             //gridY = (globalX1 * sin (-M_PI/2) - globalY1 * cos(-M_PI/2))/resolution + 49;
    //             costmap_->worldToMap(globalX1,globalY1,gridX,gridY);
    //             gridX = - gridX + 100;
    //             gridY = - gridY + 100;
    //             //RCLCPP_INFO(getLogger(), "CheckPoint..if0 grid(%d, %d)",gridX,gridY);
    //             if(gridX < mapSizeX && gridY < mapSizeY && gridX > 0 && gridY > 0)
    //             {
    //               mapOfDynamicObstacle.at<uchar>(gridX,gridY) = 254;
    //             }
    //           }
    //           if(abs(globalX2) < 2.5 && abs(globalY2) < 2.5)
    //           {
    //             //RCLCPP_INFO(getLogger(), "CheckPoint..if");
    //             costmap_->worldToMap(globalX2,globalY2,gridX,gridY);
    //             //gridX = (globalX2 * cos (-M_PI/2) + globalY2 * sin(-M_PI/2))/resolution + 49;
    //             //gridY = (globalX2 * sin (-M_PI/2) - globalY2 * cos(-M_PI/2))/resolution + 49;
    //             gridX = - gridX + 100;
    //             gridY = - gridY + 100;
    //             //RCLCPP_INFO(getLogger(), "CheckPoint..if1 grid(%d, %d)",gridX,gridY);
    //             if(gridX < mapSizeX && gridY < mapSizeY && gridX > 0 && gridY > 0)
    //             {
    //               mapOfDynamicObstacle.at<uchar>(gridX,gridY) = 254;
    //             }
    //           }
    //           //RCLCPP_INFO(getLogger(), "CheckPoint..5");
    //           if(y < round(rearBound/resolution)) 
    //           {
    //             double rearLeftRayX =  rayStep * resolution;
    //             double rearLeftRayY = -y * resolution;
    //             double rearRightRayX =  -rayStep * resolution;
    //             double rearRightRayY = -y * resolution;
    //             //RCLCPP_INFO(getLogger(), "CheckPoint..6");
    //             double globalX3 = positionOfDynamicObstacle.first + rearLeftRayX * cos(estimatedYaw) - rearLeftRayY * sin(estimatedYaw);
    //             double globalY3 = positionOfDynamicObstacle.second + rearLeftRayX * sin(estimatedYaw) + rearLeftRayY * cos(estimatedYaw);
    //             double globalX4 = positionOfDynamicObstacle.first + rearRightRayX * cos(estimatedYaw) - rearRightRayY * sin(estimatedYaw);
    //             double globalY4 = positionOfDynamicObstacle.second + rearRightRayX * sin(estimatedYaw) + rearRightRayY * cos(estimatedYaw);
    //             //double globalX3 = positionOfDynamicObstacle.first + rearLeftRayX;
    //             //double globalY3 = positionOfDynamicObstacle.second + rearLeftRayY;
    //             // double globalX4 = positionOfDynamicObstacle.first + rearRightRayX;
    //             // double globalY4 = positionOfDynamicObstacle.second + rearRightRayY;
    //             //RCLCPP_INFO(getLogger(), "CheckPoint..7");
    //           if(abs(globalX3) < 2.5 && abs(globalY3) < 2.5)
    //           {
    //             //RCLCPP_INFO(getLogger(), "CheckPoint..if");
    //             costmap_->worldToMap(globalX3,globalY3,gridX,gridY);
    //             //gridX = (globalX3 * cos (-M_PI/2) + globalY3 * sin(-M_PI/2))/resolution + 49;
    //             //gridY = (globalX3 * sin (-M_PI/2) - globalY3 * cos(-M_PI/2))/resolution + 49;
    //             gridX = - gridX + 100;
    //             gridY = - gridY + 100;
    //             //RCLCPP_INFO(getLogger(), "CheckPoint..if2 grid(%d, %d)",gridX,gridY);
    //             if(gridX < mapSizeX && gridY < mapSizeY && gridX > 0 && gridY > 0)
    //             {
    //               mapOfDynamicObstacle.at<uchar>(gridX,gridY) = 254;
    //             }
    //           }
    //           if(abs(globalX4) < 2.5 && abs(globalY4) < 2.5)
    //           {
    //             //RCLCPP_INFO(getLogger(), "CheckPoint..if");
    //             costmap_->worldToMap(globalX4,globalY4,gridX,gridY);
    //             //gridX = (globalX4 * cos (-M_PI/2) + globalY4 * sin(-M_PI/2))/resolution + 49;
    //             //gridY = (globalX4 * sin (-M_PI/2) - globalY4 * cos(-M_PI/2))/resolution + 49;
    //             gridX = - gridX + 100;
    //             gridY = - gridY + 100;
    //             //RCLCPP_INFO(getLogger(), "CheckPoint..if6 grid(%d, %d)",gridX,gridY);
    //             if(gridX < mapSizeX && gridY < mapSizeY && gridX > 0 && gridY > 0)
    //             {
    //               mapOfDynamicObstacle.at<uchar>(gridX,gridY) = 254;
    //             }
    //           }
    //           //RCLCPP_INFO(getLogger(), "CheckPoint..8");
    //           }
    //           //RCLCPP_INFO(getLogger(), "CheckPoint..9");
    //         }
          
    //     }
    //     // costmap_->worldToMap(positionOfDynamicObstacle.first,positionOfDynamicObstacle.second,positionMx,positionMy);
    //     // mapOfDynamicObstacle.at<uchar>(positionMy,positionMx) = 140;
    //     // std::random_device rd;
    //     // std::default_random_engine noise_generator;
        
    //     // // Sample from a uniform distribution i.e. [0,1)
    //     // std::uniform_real_distribution<double> uniform_dist(0,1.0);
    //     // auto skew_norm_dist_x = boost::math::skew_normal_distribution<double>(
    //     //       0, 1., 3.);
    //     // //auto skew_norm_dist_y = boost::math::skew_normal_distribution<double>(
    //     // //      0, 1., 0.);
    //     // auto skew_norm_dist_y = boost::math::normal_distribution<double>(0,0.4);
    //     // // Take a different value every time to generate probabilities from 0 to 1
    //     // for (int j = 0; j<sampleTime; j++)
    //     // {
    //     //   noise_generator.seed(rd());
    //     //   auto probability = uniform_dist(noise_generator);
    //     //   //double skew_normal_sample_point = boost::math::quantile(skew_norm_dist, probability);
    //     //   double sampleX = boost::math::quantile(skew_norm_dist_x, probability)/3.0;
    //     //   noise_generator.seed(rd());
    //     //   probability = uniform_dist(noise_generator);
          
    //     //   double sampleY = boost::math::quantile(skew_norm_dist_y, probability)/5.0;
    //     //   double inflationOfDynamicObstacleX = positionOfDynamicObstacle.first + cos(estimatedYaw) * sampleX + cos(estimatedYaw + M_PI_2)*sampleY;
    //     //   double inflationOfDynamicObstacleY = positionOfDynamicObstacle.second + sin(estimatedYaw) * sampleX + sin(estimatedYaw + M_PI_2)*sampleY;  
    //     //   //pair<double, double> InflationOfDynamicObstacle = make_pair(positionOfDynamicObstacle..first + sampling(),positionOfDynamicObstacle..second + sampling());
    //     //   unsigned int inflationMx,inflationMy;

    //     //   //RCLCPP_INFO(getLogger(),"Sample Test %.2f, %.2f",inflationOfDynamicObstacleX,inflationOfDynamicObstacleY);
    //     //   if(abs(inflationOfDynamicObstacleX) < 2.5 || abs(inflationOfDynamicObstacleY) < 2.5)
    //     //   {
            
    //     //     costmap_->worldToMap(inflationOfDynamicObstacleX,inflationOfDynamicObstacleY,inflationMx,inflationMy);
    //     //     //RCLCPP_INFO(getLogger(),"Sample Test2 %d, %d",inflationMx,inflationMy);
            
    //     //     mapOfDynamicObstacle.at<uchar>(inflationMy,inflationMx) = 254;
            
    //     //     //costmap_->setCost(inflationMx, inflationMy,50);
    //     //   }
    //     //   //RCLCPP_INFO(getLogger(),"Sample Test %.2f, %.2f",sampling(),sampling());
    //     //   //sampleList.push_back(sampling());
    //     // }
    //     //auto element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2, 2), cv::Point(1, 1));
    //     // cv::dilate(mapOfDynamicObstacle.clone(), mapOfDynamicObstacle, cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(2,2),cv::Point(1,1)));
    //     //cv::erode(mapOfDynamicObstacle.clone(),mapOfDynamicObstacle,cv::getStructuringElement(cv::MORPH_ERODE,cv::Size(3,3),cv::Point(2,2)));
    //     //cv::morphologyEx(mapOfDynamicObstacle.clone(),mapOfDynamicObstacle,cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_OPEN, cv::Size(3, 3)));
    //    // cv::erode(mapOfDynamicObstacle.clone(),mapOfDynamicObstacle,cv::getStructuringElement(cv::MORPH_ERODE,cv::Size(3,3),cv::Point(2,2)));
    //     //cv::dilate(mapOfDynamicObstacle.clone(), mapOfDynamicObstacle, element);
    //     //element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3), cv::Point(2, 2));
    //     //cv::dilate(mapOfDynamicObstacle.clone(), mapOfDynamicObstacle.clone(), element);
        
    //     //cv::GaussianBlur(mapOfDynamicObstacle.clone(), mapOfDynamicObstacle,  cv::Size(5, 5),0);
        
    //     //cv::namedWindow("img3");
        
    //     // cv::imshow("img3",mapOfDynamicObstacle);
    //     // cv::waitKey(1);
    //   }
    // }
    //example to visualize filtered unknown static obstacle

    // if(obstacleId.size())
    // {  
    //   std::sort(obstacleId.begin(), obstacleId.end(),[](int a, int b){return a<b;});
    //   RCLCPP_INFO(getLogger(),"Erase unknown static obstacle(trivial velocity) for visualization");
    //   for(int i = obstacleId.size()-1; i>-1; i--)
    //   {
    //     RCLCPP_INFO(getLogger(), "Erase CheckPoint..");
    //     polygons->erase(polygons->begin()+obstacleId[i]);
    //     RCLCPP_INFO(getLogger(), "Erase CheckPoint..0");
    //     obstacleId.erase(obstacleId.begin()+i);
    //     RCLCPP_INFO(getLogger(), "Erase CheckPoint..1");
    //     //RCLCPP_INFO(getLogger(),"remain number of obstacle %d", obstacleId.size());
    //   }
    // }
    // replace shared polygon container


    
    //need tracker Container update function 
    updatePolygonContainer(polygons);
    //RCLCPP_INFO(getLogger(), "polygon size %d",polygons->size());
}

void CostmapToPolygonsDBSMCCH::setCostmap2D(nav2_costmap_2d::Costmap2D *costmap)

{
    if (!costmap)
      return;
    trackers_ = std::make_shared<std::vector<KalmanEigen>>();
    costmap_ = costmap;
    updateCostmap2D();

}

void CostmapToPolygonsDBSMCCH::updateCostmap2D()
{
      occupied_cells_.clear();

      if (!costmap_->getMutex())
      {
        RCLCPP_ERROR(getLogger(), "Cannot update costmap since the mutex pointer is null");
        return;
      }
      
      // TODO: currently dynamic reconigure is not supported in ros2
      { // get a copy of our parameters from dynamic reconfigure
        std::lock_guard<std::mutex> lock(parameter_mutex_);
        parameter_ = parameter_buffered_;
      }

      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap_->getMutex());
      
      // allocate neighbor lookup
      int cells_x = int(costmap_->getSizeInMetersX() / parameter_.max_distance_) + 1;
      int cells_y = int(costmap_->getSizeInMetersY() / parameter_.max_distance_) + 1;

      if (cells_x != neighbor_size_x_ || cells_y != neighbor_size_y_) {
        neighbor_size_x_ = cells_x;
        neighbor_size_y_ = cells_y;
        neighbor_lookup_.resize(neighbor_size_x_ * neighbor_size_y_);
      }
      offset_x_ = costmap_->getOriginX();
      offset_y_ = costmap_->getOriginY();
      for (auto& n : neighbor_lookup_)
        n.clear();

      auto layers = costmap_ros_->getLayeredCostmap()->getPlugins();
      cv::Mat costmapMat;
      cv::Mat inscribeCostmap;
      cv::Mat element2;
      
      unsigned int costMapSizeX = costmap_->getSizeInCellsX();
      unsigned int costMapSizeY = costmap_->getSizeInCellsY();
      double costMapOriginX = costmap_->getOriginX();
      double costMapOriginY = costmap_->getOriginY();
      nav2_costmap_2d::Costmap2D static_costmap = nav2_costmap_2d::Costmap2D(costMapSizeX,costMapSizeY, costmap_->getResolution(),costMapOriginX, costMapOriginY);
      // double vertX,vertY;
      // double SizeX = static_cast<double>(costMapSizeX);
      // double SizeY = static_cast<double>(costMapSizeY);
      // double OriginX = 0.0;
      // double OriginY = 0.0;
      for (auto it = layers->begin(); it != layers->end(); it++)
      {
        st_ptr = std::dynamic_pointer_cast<nav2_costmap_2d::StaticLayer>(*it);
        if(!st_ptr)
          continue;
        //std::unique_lock<nav2_costmap_2d::StaticLayer::mutex_t> master_lock(st_ptr);
      
        //st_ptr->onInitialize();
        //st_ptr->reset();
        //RCLCPP_INFO(getLogger(),"Set StaticLayer");

        //RCLCPP_INFO(getLogger(), "Origin(%.2f,%.2f) Size(%d,%d)",costMapOriginX,costMapOriginY,costMapSizeX,costMapSizeY);
        
        //static_costmap = std::make_shared<nav2_costmap_2d::Costmap2D>(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY(), costmap_->getResolution(), costmap_->getOriginX(), costmap_->getOriginY());
        //static_costmap->resetMap(0,0,static_costmap->getSizeInCellsX(), static_costmap->getSizeInCellsY());
        //RCLCPP_INFO(getLogger(),"Set1");
        //costmap_->mapToWorld(0,0,vertX,vertY);
        //costmap_->mapToWorld(costMapSizeX,costMapSizeY,vertX,vertY);
        //RCLCPP_INFO(getLogger(),"Set2-0");
        //st_ptr->onInitialize();
        //st_ptr->updateBounds(0,0,0,&OriginX,&OriginY,&SizeX,&SizeY);
        //RCLCPP_INFO(getLogger(),"Set2-1");
        //static_costmap.resetMap(0,0,costMapSizeX,costMapSizeY);
        st_ptr->updateCosts(static_costmap, 0, 0, costMapSizeX, costMapSizeY);
        //RCLCPP_INFO(getLogger(),"Set2-2");
        costmapMat = cv::Mat(costMapSizeY, costMapSizeX, CV_8UC1, static_costmap.getCharMap()).clone();
        //RCLCPP_INFO(getLogger(), "Size 1 (%d,%d) Size 2 (%d,%d)",costmap_->getSizeInCellsX(),costmap_->getSizeInCellsY(),costmapMat.size().width,costmapMat.size().height);
        element2 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5), cv::Point(3, 3));
        cv::dilate(costmapMat.clone(), inscribeCostmap, element2);
        //RCLCPP_INFO(getLogger(),"Set2-3");
        //RCLCPP_INFO(getLogger(), "Size 1 (%d,%d) Size 2 (%d,%d)",costmap_->getSizeInCellsX(),costmap_->getSizeInCellsY(),inscribeCostmap.size().width,inscribeCostmap.size().height);
        cv::GaussianBlur(inscribeCostmap, inscribeCostmap,  cv::Size(5, 5),0);
        // cv::namedWindow("img1");
        // cv::namedWindow("img2");
        // cv::imshow("img1",costmapMat);
        // cv::imshow("img2",inscribeCostmap);
        // cv::waitKey(1);
        //RCLCPP_INFO(getLogger(),"Set2-4");
      }
      
      //RCLCPP_INFO(getLogger(),"Set3");
      //std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock2(*static_costmap->getMutex());
      //if(st_ptr)
      //  st_ptr->updateCosts(*static_costmap, 0, 0, static_costmap->getSizeInCellsX(), static_costmap->getSizeInCellsY());
      
      //RCLCPP_INFO(getLogger(), "Size(%d,%d)",costmap_->getSizeInCellsX(),costmap_->getSizeInCellsY());
      //cv::Mat element2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5), cv::Point(3, 3));
      //cv::dilate(costmapMat, inscribeCostmap, element2);
      //cv::GaussianBlur(costmapMat, inscribeCostmap,  cv::Size(7, 7),0);
      for(int i = 0; i < costmap_->getSizeInCellsX(); i++)
      {
        for(int j = 0; j < costmap_->getSizeInCellsY(); j++)
        {
          //RCLCPP_INFO(getLogger(),"Set4");
          int value = costmap_->getCost(i,j);
          //int staticValue = static_costmap.getCost(i,j);
          int staticValue = inscribeCostmap.at<uchar>(j,i);
          //RCLCPP_INFO(getLogger(),"value %d, staticValue %d",value,staticValue);
          if(staticValue >0)
             continue;
          // //int staticValue = static_costmap->getCost(i,j);
          // //int inscribe_radius = 200;
          // //int resolution = 50;
          // //double wx,wy;
          // //RCLCPP_INFO(getLogger(),"Set2");
          // //static_costmap->mapToWorld(i,j,wx,wy);
          // //RCLCPP_INFO(getLogger(),"Set3 %d %d -> %.2f %.2f",i,j,wx,wy);
          //RCLCPP_INFO(getLogger(),"Cost(%d,%d) : %d",i,j,value);
          // // int wxInt = wx*1000;
          // // int wyInt = wy*1000;
          // // unsigned int inscribeMx = 0;
          // // unsigned int inscribeMy = 0;
          // // bool occupied = false;
          // //   for(int angle = 0; angle<360; angle++)
          // //   {
          // //     double rad = angle * 3.1415926/180.0;
          // //     if(!occupied)
          // //     {
          // //       for(int n = 0; n<=inscribe_radius; n=n+resolution)
          // //       {
          // //         double inscribeWx = (wxInt+n*cos(rad))/1000.0;
          // //         double inscribeWy = (wyInt+n*sin(rad))/1000.0;
                  
          // //         static_costmap->worldToMap(inscribeWx,inscribeWy,inscribeMx,inscribeMy);
          // //         //RCLCPP_INFO(getLogger(),"Set3 %.2f %.2f -> %d %d",inscribeWx,inscribeWy,inscribeMx,inscribeMy);
          // //         int inscribeValue = static_costmap->getCost(inscribeMx,inscribeMy);
          // //         //RCLCPP_INFO(getLogger(),"Cost %d",inscribeValue);
          // //         if(inscribeMx > 60 || inscribeMy > 60)
          // //           continue;
          // //         if(inscribeValue>0)
          // //         {  
          // //           RCLCPP_INFO(getLogger(),"Set4-1");
          // //           occupied= true;
          // //         }
          // //       }
          // //     }
          // //     else
          // //       break;
          // //   }
          // // RCLCPP_INFO(getLogger(),"Set5");
          // // if(occupied)
          // //   continue;


          // double searchDistance = 0.2;
          // double resolution 0.05;
          // int searchPixel = int(searchDistance/resolution)
          // for(int n = 0; n <searchPixel; n++)
          // {}
          //int staticValue = StaticCostmap->getCost(i,j);
          //RCLCPP_INFO(getLogger(),"Set6 Value %d",staticValue);
          if(value >= nav2_costmap_2d::LETHAL_OBSTACLE && staticValue !=value)
          {
            //RCLCPP_INFO(getLogger(),"(%d,%d) Value %d Static value %d",i,j,value,staticValue);
            double x, y;
            costmap_->mapToWorld((unsigned int)i, (unsigned int)j, x, y);
            addPoint(x, y);
          }
        }
      }
}


void CostmapToPolygonsDBSMCCH::dbScan(std::vector< std::vector<KeyPoint> >& clusters)
{
  std::vector<bool> visited(occupied_cells_.size(), false);

  clusters.clear();

  //DB Scan Algorithm
  int cluster_id = 0; // current cluster_id
  clusters.push_back(std::vector<KeyPoint>());
  for(int i = 0; i< (int)occupied_cells_.size(); i++)
  {
    if(!visited[i]) //keypoint has not been visited before
    {
      visited[i] = true; // mark as visited
      std::vector<int> neighbors;
      regionQuery(i, neighbors); //Find neighbors around the keypoint
      if((int)neighbors.size() < parameter_.min_pts_) //If not enough neighbors are found, mark as noise
      {
        clusters[0].push_back(occupied_cells_[i]);
      }
      else
      {
        ++cluster_id; // increment current cluster_id
        clusters.push_back(std::vector<KeyPoint>());

        // Expand the cluster
        clusters[cluster_id].push_back(occupied_cells_[i]);
        for(int j = 0; j<(int)neighbors.size(); j++)
        {
          if ((int)clusters[cluster_id].size() == parameter_.max_pts_)
            break;

          if(!visited[neighbors[j]]) //keypoint has not been visited before
          {
            visited[neighbors[j]] = true;  // mark as visited
            std::vector<int> further_neighbors;
            regionQuery(neighbors[j], further_neighbors); //Find more neighbors around the new keypoint
//             if(further_neighbors.size() < min_pts_)
//             {
//               clusters[0].push_back(occupied_cells[neighbors[j]]);
//             }
//             else
            if ((int)further_neighbors.size() >= parameter_.min_pts_)
            {
              // neighbors found
              neighbors.insert(neighbors.end(), further_neighbors.begin(), further_neighbors.end());  //Add these newfound P' neighbour to P neighbour vector "nb_indeces"
              clusters[cluster_id].push_back(occupied_cells_[neighbors[j]]);
            }
          }
        }
      }
    }
  }
}

void CostmapToPolygonsDBSMCCH::regionQuery(int curr_index, std::vector<int>& neighbors)
{
    neighbors.clear();

    double dist_sqr_threshold = parameter_.max_distance_ * parameter_.max_distance_;
    const KeyPoint& kp = occupied_cells_[curr_index];
    int cx, cy;
    pointToNeighborCells(kp, cx,cy);

    // loop over the neighboring cells for looking up the points
    const int offsets[9][2] = {{-1, -1}, {0, -1}, {1, -1},
                               {-1,  0}, {0,  0}, {1,  0},
                               {-1,  1}, {0,  1}, {1,  1}};
    for (int i = 0; i < 9; ++i)
    {
      int idx = neighborCellsToIndex(cx + offsets[i][0], cy + offsets[i][1]);
      if (idx < 0 || idx >= int(neighbor_lookup_.size()))
        continue;
      const std::vector<int>& pointIndicesToCheck = neighbor_lookup_[idx];
      for (int point_idx : pointIndicesToCheck) {
        if (point_idx == curr_index) // point is not a neighbor to itself
          continue;
        const KeyPoint& other = occupied_cells_[point_idx];
        double dx = other.x - kp.x;
        double dy = other.y - kp.y;
        double dist_sqr = dx*dx + dy*dy;
        if (dist_sqr <= dist_sqr_threshold)
          neighbors.push_back(point_idx);
      }
    }
}

bool isXCoordinateSmaller(const CostmapToPolygonsDBSMCCH::KeyPoint& p1, const CostmapToPolygonsDBSMCCH::KeyPoint& p2)
{
  return p1.x < p2.x || (p1.x == p2.x && p1.y < p2.y);
}

void CostmapToPolygonsDBSMCCH::convexHull(std::vector<KeyPoint>& cluster, geometry_msgs::msg::Polygon& polygon)
{
    //Monotone Chain ConvexHull Algorithm source from http://www.algorithmist.com/index.php/Monotone_Chain_Convex_Hull

    int k = 0;
    int n = cluster.size();

    // sort points according to x coordinate (TODO. is it already sorted due to the map representation?)
    std::sort(cluster.begin(), cluster.end(), isXCoordinateSmaller);

    polygon.points.resize(2*n);

    // lower hull
    for (int i = 0; i < n; ++i)
    {
      while (k >= 2 && cross(polygon.points[k-2], polygon.points[k-1], cluster[i]) <= 0)
      {
        --k;
      }
      cluster[i].toPointMsg(polygon.points[k]);
      ++k;
    }

    // upper hull
    for (int i = n-2, t = k+1; i >= 0; --i)
    {
      while (k >= t && cross(polygon.points[k-2], polygon.points[k-1], cluster[i]) <= 0)
      {
        --k;
      }
      cluster[i].toPointMsg(polygon.points[k]);
      ++k;
    }


    polygon.points.resize(k); // original
    // TEST we skip the last point, since in our definition the polygon vertices do not contain the start/end vertex twice.
//     polygon.points.resize(k-1); // TODO remove last point from the algorithm above to reduce computational cost

    simplifyPolygon(polygon);
}



void CostmapToPolygonsDBSMCCH::convexHull2(std::vector<KeyPoint>& cluster, geometry_msgs::msg::Polygon& polygon)
{
    std::vector<KeyPoint>& P = cluster;
    std::vector<geometry_msgs::msg::Point32>& points = polygon.points;

    // Sort P by x and y
    std::sort(P.begin(), P.end(), isXCoordinateSmaller);

    // the output array H[] will be used as the stack
    int i;                 // array scan index

    // Get the indices of points with min x-coord and min|max y-coord
    int minmin = 0, minmax;
    double xmin = P[0].x;
    for (i = 1; i < (int)P.size(); i++)
        if (P[i].x != xmin) break;
    minmax = i - 1;
    if (minmax == (int)P.size() - 1)
    {   // degenerate case: all x-coords == xmin
        points.push_back(geometry_msgs::msg::Point32());
        P[minmin].toPointMsg(points.back());
        if (P[minmax].y != P[minmin].y) // a  nontrivial segment
        {
            points.push_back(geometry_msgs::msg::Point32());
            P[minmax].toPointMsg(points.back());
        }
        // add polygon endpoint
        points.push_back(geometry_msgs::msg::Point32());
        P[minmin].toPointMsg(points.back());
        return;
    }

    // Get the indices of points with max x-coord and min|max y-coord
    int maxmin, maxmax = (int)P.size() - 1;
    double xmax = P.back().x;
    for (i = P.size() - 2; i >= 0; i--)
        if (P[i].x != xmax) break;
    maxmin = i+1;

    // Compute the lower hull on the stack H
    // push  minmin point onto stack
    points.push_back(geometry_msgs::msg::Point32());
    P[minmin].toPointMsg(points.back());
    i = minmax;
    while (++i <= maxmin)
    {
        // the lower line joins P[minmin]  with P[maxmin]
        if (cross(P[minmin], P[maxmin], P[i]) >= 0 && i < maxmin)
            continue;           // ignore P[i] above or on the lower line

        while (points.size() > 1)         // there are at least 2 points on the stack
        {
            // test if  P[i] is left of the line at the stack top
            if (cross(points[points.size() - 2], points.back(), P[i]) > 0)
                break;         // P[i] is a new hull  vertex
            points.pop_back();         // pop top point off  stack
        }
        // push P[i] onto stack
        points.push_back(geometry_msgs::msg::Point32());
        P[i].toPointMsg(points.back());
    }

    // Next, compute the upper hull on the stack H above  the bottom hull
    if (maxmax != maxmin)      // if  distinct xmax points
    {
         // push maxmax point onto stack
         points.push_back(geometry_msgs::msg::Point32());
         P[maxmax].toPointMsg(points.back());
    }
    int bot = (int)points.size();                  // the bottom point of the upper hull stack
    i = maxmin;
    while (--i >= minmax)
    {
        // the upper line joins P[maxmax]  with P[minmax]
        if (cross( P[maxmax], P[minmax], P[i])  >= 0 && i > minmax)
            continue;           // ignore P[i] below or on the upper line

        while ((int)points.size() > bot)     // at least 2 points on the upper stack
        {
            // test if  P[i] is left of the line at the stack top
            if (cross(points[points.size() - 2], points.back(), P[i]) > 0)
                break;         // P[i] is a new hull  vertex
            points.pop_back();         // pop top point off stack
        }
        // push P[i] onto stack
        points.push_back(geometry_msgs::msg::Point32());
        P[i].toPointMsg(points.back());
    }
    if (minmax != minmin)
    {
        // push  joining endpoint onto stack
        points.push_back(geometry_msgs::msg::Point32());
        P[minmin].toPointMsg(points.back());
    }

    simplifyPolygon(polygon);
}

void CostmapToPolygonsDBSMCCH::simplifyPolygon(geometry_msgs::msg::Polygon& polygon)
{
  size_t triangleThreshold = 3;
  // check if first and last point are the same. If yes, a triangle has 4 points
  if (polygon.points.size() > 1
      && std::abs(polygon.points.front().x - polygon.points.back().x) < 1e-5
      && std::abs(polygon.points.front().y - polygon.points.back().y) < 1e-5)
  {
    triangleThreshold = 4;
  }
  if (polygon.points.size() <= triangleThreshold) // nothing to do for triangles or lines
    return;
  // TODO Reason about better start conditions for splitting lines, e.g., by
  // https://en.wikipedia.org/wiki/Rotating_calipers
  polygon.points = douglasPeucker(polygon.points.begin(), polygon.points.end(), parameter_.min_keypoint_separation_);
}

void CostmapToPolygonsDBSMCCH::updatePolygonContainer(PolygonContainerPtr polygons)
{
  std::lock_guard<std::mutex> lock(mutex_);
  polygons_ = polygons;
}
// void CostmapToPolygonsDBSMCCH::updateTrackerContainer(TrackerContainerPtr trackers)
// {
//   std::lock_guard<std::mutex> lock(mutex_);
//   trackers_ = trackers;
// }
TrackerContainerPtr CostmapToPolygonsDBSMCCH::getTrackers()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return trackers_;
}
PolygonContainerConstPtr CostmapToPolygonsDBSMCCH::getPolygons()
{
  std::lock_guard<std::mutex> lock(mutex_);
  PolygonContainerConstPtr polygons = polygons_;
  return polygons;
}

//void CostmapToPolygonsDBSMCCH::reconfigureCB(CostmapToPolygonsDBSMCCHConfig& config, uint32_t level)
//{
  //boost::mutex::scoped_lock lock(parameter_mutex_);
  //parameter_buffered_.max_distance_ = config.cluster_max_distance;
  //parameter_buffered_.min_pts_ = config.cluster_min_pts;
  //parameter_buffered_.max_pts_ = config.cluster_max_pts;
  //parameter_buffered_.min_keypoint_separation_ = config.convex_hull_min_pt_separation;
//}

} //end namespace costmap_converter


