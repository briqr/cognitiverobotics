// %Tag(FULLTEXT)%
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <pcl_ros/point_cloud.h>
#include <pcl/common/distances.h>
#include <pcl/point_types.h>
#include "../include/cognitive_robotics/point_type.h"

#include <boost/circular_buffer.hpp>

#include <cognitive_robotics/point_type.h>



ros::Publisher g_pub;
double threshold = 10.0;
const int NUM_RINGS = 16;
const int WINDOW_SIZE = 10;
//std::map<uint16_t, boost::circular_buffer<velodyne_pointcloud::PointXYZIR> > ringPointMap;
 
// A struct that represents the average distance of some points in a predefined windows and contains the points themselves as well
 struct PointAverageDistance 
  {
    PointAverageDistance()
    : pointsBuffer(10)
    {
      
    }
    double averageNorm;
    boost::circular_buffer<velodyne_pointcloud::PointXYZIR> pointsBuffer;
  } ;


std::vector <std::map<uint16_t,std::vector <std::vector<velodyne_pointcloud::PointXYZIR > > > > obstaclesList;

double distance(const velodyne_pointcloud::PointXYZIR& point1, const velodyne_pointcloud::PointXYZIR& point2) {
  return  pcl::euclideanDistance<velodyne_pointcloud::PointXYZIR>(point1, point2);
}

void updatePointsBuffer(PointAverageDistance & pointsBufferAvg, const velodyne_pointcloud::PointXYZIR newPoint){
  double averageNorm = pointsBufferAvg.averageNorm;
  velodyne_pointcloud::PointXYZIR pointToOverwrite = pointsBufferAvg.pointsBuffer.front();
  pointsBufferAvg.pointsBuffer.push_back(newPoint);
  double newAverageNorm = (averageNorm*WINDOW_SIZE - (newPoint.getVector4fMap()).norm() +  (newPoint.getVector4fMap()).norm())/WINDOW_SIZE;
  pointsBufferAvg.averageNorm = newAverageNorm;
}
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
// %Tag(CALLBACK)%
void subscriberCallback(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>& cloud_)
{
  
  ROS_INFO("callback");
  //sensor_msgs::PointCloud
  pcl::PointCloud<velodyne_pointcloud::PointXYZIR> cloud = cloud_;
  pcl::PointCloud<velodyne_pointcloud::PointXYZIR> modifiedCloud;
  modifiedCloud.header = cloud_.header;
  std::vector< velodyne_pointcloud::PointXYZIR, Eigen::aligned_allocator <velodyne_pointcloud::PointXYZIR> > ::iterator iter = cloud.points.begin();
  std::map<uint16_t, std::vector <std::vector<velodyne_pointcloud::PointXYZIR> > > obstacles;
  
  // a map between a ring and the last point seen so far
  std::map<uint16_t,  PointAverageDistance> ringPointMap;
  
  //indicates if the last point in the ring inspected was an obstacle
  std::map<uint16_t, bool> obstaclesIndicator;
  
  for(int i =1; i <= NUM_RINGS; i++) {
   obstaclesIndicator[i] = false; 
  }
  
  for (; iter != cloud.points.end(); iter++) {
    velodyne_pointcloud::PointXYZIR currentPoint = *iter;
    if(ringPointMap.count(currentPoint.ring) < 1) {
      //currentPoint.intensity = (currentPoint.getVector4fMap()).norm();
      PointAverageDistance pointCountAvg;
      pointCountAvg.pointsBuffer.push_back(currentPoint);
      pointCountAvg.averageNorm = (currentPoint.getVector4fMap()).norm();
      ringPointMap[currentPoint.ring] = pointCountAvg;
      continue;
    }
    PointAverageDistance previousPoints = ringPointMap[currentPoint.ring];
    double averageNorm = previousPoints.averageNorm;
    double currentNorm = (currentPoint.getVector4fMap()).norm();
    //this is the first point in the obstacle
    // this point belongs to a new obstacle
    if(currentNorm-averageNorm > threshold && obstaclesIndicator[currentPoint.ring]==false) {
      int obstacleNum = (obstacles[currentPoint.ring]).size(); // start a new obstacle vector
      ((obstacles[currentPoint.ring])[obstacleNum]).push_back(currentPoint);
      obstaclesIndicator[currentPoint.ring] = true;
      modifiedCloud.push_back(currentPoint);
    }
    // this point belongs to a previously detected obstacle
    else  if(currentNorm-averageNorm < threshold && obstaclesIndicator[currentPoint.ring]==true) {
      int obstacleNum = (obstacles[currentPoint.ring]).size(); // append to the last obstacle vector
      ((obstacles[currentPoint.ring])[obstacleNum-1]).push_back(currentPoint);
      modifiedCloud.push_back(currentPoint);
    }
    // the last obstacle for this ring ends here
    else  if(currentNorm-averageNorm > threshold && obstaclesIndicator[currentPoint.ring]==true) {
      obstaclesIndicator[currentPoint.ring]==false;
    }
 
   updatePointsBuffer(previousPoints, currentPoint);
  } 
   obstaclesList.push_back(obstacles);
   g_pub.publish(modifiedCloud);  
  
}
// %EndTag(CALLBACK)%

int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "listener");

  
  
  ros::NodeHandle n;


// %Tag(SUBSCRIBER)%
  ros::Subscriber sub = n.subscribe("velodyne_points", 1000, subscriberCallback); // topic name: velodyne_points

  g_pub = n.advertise<pcl::PointCloud<velodyne_pointcloud::PointXYZIR> > ("output",1);
// %EndTag(SUBSCRIBER)%

// %Tag(SPIN)%
  ros::spin();
// %EndTag(SPIN)%

  return 0;
}
// %EndTag(FULLTEXT)%
