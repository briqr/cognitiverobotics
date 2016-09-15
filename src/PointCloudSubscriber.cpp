// %Tag(FULLTEXT)%
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

#include <pcl_ros/point_cloud.h>
#include <pcl/common/distances.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include "../include/cognitive_robotics/point_type.h"

#include <boost/circular_buffer.hpp>

#include <cognitive_robotics/point_type.h>
#include <cognitive_robotics/utility.hpp>



ros::Publisher g_pub;
ros::Publisher g_pub_median;
double threshold = 0.5;
const int NUM_RINGS = 16;
const int WINDOW_SIZE = 500;
std::vector <std::map<uint16_t,std::vector <std::vector<velodyne_pointcloud::PointXYZIR > > > > obstaclesList;
// the angle between 2 consecutive laser diodes
double diodeAngle = 2;
double epsilon = 0.05;
int scan_nr = 0;

std::map<uint16_t, double > g_medianFactorByRing;
 
// A struct that represents the average distance of some points in a predefined windows and contains the points themselves as well
 struct PointAverageDistance 
  {
    PointAverageDistance() : pointsBuffer(WINDOW_SIZE){}
    double averageNorm;
    boost::circular_buffer<velodyne_pointcloud::PointXYZIR> pointsBuffer;
  } ;


double distance(const velodyne_pointcloud::PointXYZIR& point1, const velodyne_pointcloud::PointXYZIR& point2) {
  return  pcl::euclideanDistance<velodyne_pointcloud::PointXYZIR>(point1, point2);
}



void getRainbowColor(float value, float& r, float& g, float& b){
    // this is HSV color palette with hue values going only from 0.0 to 0.833333.
    value = std::min(value, 1.0f);
    value = std::max(value, 0.0f);

    float color[3];

    float h = value * 5.0f + 1.0f;
    int i = floor(h);
    float f = h - i;
    if (!(i & 1))
    f = 1 - f; // if i is even
    float n = 1 - f;

    if (i <= 1)
    color[0] = n, color[1] = 0, color[2] = 1;
    else if (i == 2)
    color[0] = 0, color[1] = n, color[2] = 1;
    else if (i == 3)
    color[0] = 0, color[1] = 1, color[2] = n;
    else if (i == 4)
    color[0] = n, color[1] = 1, color[2] = 0;
    else if (i >= 5)
    color[0] = 1, color[1] = n, color[2] = 0;

    r = color[0];
    g = color[1];
    b = color[2];
  }






double expectedInterRingDistance(const velodyne_pointcloud::PointXYZIR point1, const velodyne_pointcloud::PointXYZIR point2)  {
  double norm1 = (point1.getVector4fMap()).norm();
  double norm2 = (point2.getVector4fMap()).norm();
  double angle = fabs(point2.ring-point1.ring)*diodeAngle;
  //double expectedDistance = pow(norm1, 2.0)+pow(norm2, 2.0)-2*norm1*norm2*cos ( angle * M_PI / 180.0 );
  double expectedDistance = norm1*cos ( angle * M_PI / 180.0 );
  //std::cout << "***** expected distance " << expectedDistance << "*****" << std::endl;
  
//   return sqrt(expectedDistance);
  return (expectedDistance);
}


 double distanceBetweenClustersSingleLink(const std::vector<velodyne_pointcloud::PointXYZIR>& clusteri, const std::vector<velodyne_pointcloud::PointXYZIR>& clusterj) {
   int n = clusteri.size(), m = clusterj.size();
   double minDistanceBetweenClusters = std::numeric_limits<double>::max();
   for(int i =0; i<n; i++) {
    for(int j =0; j<m; j++) {  
	double currentDistance = pcl::euclideanDistance<velodyne_pointcloud::PointXYZIR>(clusteri[i], clusterj[j]);
	if(currentDistance < minDistanceBetweenClusters) {
	  minDistanceBetweenClusters = currentDistance;
	}
    }
  }
  return minDistanceBetweenClusters;
 }
 
 

 std::vector<std::vector<velodyne_pointcloud::PointXYZIR> > clusterObstaclesSingleLink(const std::vector<velodyne_pointcloud::PointXYZIR>& obstacles, int k) {
  // double clusterDistanceThreshold = 0.0001;
   //pcl::StopWatch timer;
   int initClusterSize = obstacles.size();
   int numClusters = initClusterSize;
   k = 100;
 
   int* minDistanceIndices = new int[initClusterSize];
   double* minDistanceValues = new double[initClusterSize];
   double** clusterDistances = new double*[initClusterSize];
   for(int i = 0; i < initClusterSize; ++i) {
      clusterDistances[i] = new double[initClusterSize];
   }
   std::vector<std::vector<velodyne_pointcloud::PointXYZIR> > currentClustering(obstacles.size());
   for(int i =0; i < numClusters; i++) { // the initial cluster contains each point from the dataset as a cluster
    //std:: cout<< "pushing back: " << obstacles[i].x<<",";
     currentClustering[i].push_back(obstacles[i]);
   }
   //std::cout<< "******beginning clustering size: "<< currentClustering.size();
   int minIndexi, minIndexj;
   //std::cout << "reached here!!!!!!!!!" <<std::endl;
   for(int i = 0; i < initClusterSize; ++i) {
      minDistanceValues[i] =  std::numeric_limits<double>::infinity();
   }
   
   while(numClusters > k) { // merge cluster based on the closest distance
    if(numClusters == initClusterSize) {// this is the first iteration
      for(int i =0; i<numClusters; i++) {
	clusterDistances[i][i] = std::numeric_limits<double>::infinity();
	  for (int j=i+1; j<numClusters ;j++) {
	    double currentDistance= distanceBetweenClustersSingleLink(currentClustering[i], currentClustering[j]);
	    if(currentDistance < 0.000001)
	      std::cout <<"1****i, j distance: (" <<i << ", "<< j <<", " << currentDistance << ", " <<")" <<",";
	    clusterDistances[i][j] = currentDistance;
	    clusterDistances[j][i] = currentDistance;
	    if(currentDistance < minDistanceValues[i]) {
	      minDistanceIndices[i]=j;
	      minDistanceValues[i]=currentDistance;
	    }
	}
     }
   }
    //find the minimum distance
    double minDistance = std::numeric_limits<double>::infinity();
    for(int i =0; i<initClusterSize; i++) {
	double currentDistance= minDistanceValues[i];
	    if(currentDistance < minDistance) {
	      minDistance = currentDistance;
	      minIndexi = i;
	      minIndexj = minDistanceIndices[i];
	    }
	}
	
    std::cout << "----------------------min distance: " << minDistance;
      //update the affected distances as a result of merging
    minDistance = std::numeric_limits<double>::infinity();
    for(int i =0; i<initClusterSize; i++) {
      if(i==minIndexi) {
	continue;
      }
      if(i != minIndexj) {
	clusterDistances[minIndexi][i] = std::min(clusterDistances[minIndexi][i], clusterDistances[minIndexj][i]);
	clusterDistances[i][minIndexi] =  clusterDistances[minIndexi][i];
	if(clusterDistances[minIndexi][i] < minDistance) {
	  minDistance = clusterDistances[minIndexi][i];
	  minDistanceValues[minIndexi] = minDistance;
	  minDistanceIndices[minIndexi] = i;
      }
    }
     clusterDistances[minIndexj][i] = std::numeric_limits<double>::infinity();
     clusterDistances[i][minIndexj] = std::numeric_limits<double>::infinity();
      
     if(minDistanceIndices[i] == minIndexj) {
	minDistanceIndices[i] = minIndexi;
      }
    }
    minDistanceValues[minIndexj] = std::numeric_limits<double>::infinity();
    clusterDistances[minIndexj][minIndexi] = std::numeric_limits<double>::infinity();
    clusterDistances[minIndexi][minIndexj] = std::numeric_limits<double>::infinity();
   
    // merge the clusters
    std::vector<velodyne_pointcloud::PointXYZIR> mergedCluster;
    mergedCluster.reserve(currentClustering[minIndexi].size() + currentClustering[minIndexj].size() ); 
     mergedCluster.insert(mergedCluster.end(), currentClustering[minIndexi].begin(), currentClustering[minIndexi].end());
    mergedCluster.insert(mergedCluster.end(), currentClustering[minIndexj].begin(), currentClustering[minIndexj].end());

      //std::cout << "coutmergedCluster size:" << mergedCluster.size() <<  std::endl;
    std::vector<velodyne_pointcloud::PointXYZIR>().swap(currentClustering[minIndexi]);
    std::vector<velodyne_pointcloud::PointXYZIR>().swap(currentClustering[minIndexj]);
    currentClustering[minIndexi] = mergedCluster;


    --numClusters;
  }

    //std::cout << "after merge: " << currentClustering.size() <<std::endl;  
    for(int i = 0; i < initClusterSize; ++i) {
      delete [] clusterDistances[i];
   }
      delete []clusterDistances;

   delete [] minDistanceIndices;
   delete [] minDistanceValues;
       
  // assign id to clusters
   //std::cout<< "total num points: (" << initClusterSize <<"), ";
   int clusterId = 0;
  for(int i =0; i<currentClustering.size(); i++) {
    if(currentClustering[i].size()<1)
      continue;
    //std::cout<< "num points in cluster: (" << currentClustering[i].size() <<"), ";
    float c, r, g, b;
    c = clusterId*2.5 / (float)k;
    //ROS_INFO_STREAM( "clusterid: " << clusterId << " " << c ); 
    getRainbowColor(c,r,g,b);
    for(int j=0; j<currentClustering[i].size();j++) {
	(currentClustering[i])[j].cluster_id = clusterId;
	//uint32_t rgb = ((uint32_t)22 << 16 | (uint32_t)clusterId << 8 | (uint32_t)10);
	(currentClustering[i])[j].r = r*255;
	(currentClustering[i])[j].g = g*255;
	(currentClustering[i])[j].b = b*255;
    }
    ++clusterId;
  }
  //std::cout<< "******end clustering size: "<< currentClustering[0].size() << std::endl; 
  //std::cout << "time:" << timer.getTime() << std::endl;
  return currentClustering;
 }
 

void updatePointsBuffer(PointAverageDistance & pointsBufferAvg, const velodyne_pointcloud::PointXYZIR newPoint){
  double averageNorm = pointsBufferAvg.averageNorm;
  velodyne_pointcloud::PointXYZIR pointToOverwrite = pointsBufferAvg.pointsBuffer.front();
  pointsBufferAvg.pointsBuffer.push_back(newPoint);
  int numPoints =  pointsBufferAvg.pointsBuffer.size();
  
  
  double newAverageNorm; 
  if ( numPoints < WINDOW_SIZE )
  {
    newAverageNorm = (averageNorm*(numPoints-1) +  (newPoint.getVector4fMap()).norm())/(numPoints);  
  }
  else 
  {
    newAverageNorm = (averageNorm*(numPoints) - (pointToOverwrite.getVector4fMap()).norm() +  (newPoint.getVector4fMap()).norm())/(numPoints);
    
    if (newPoint.ring == 10 )
      ROS_INFO_STREAM("updatePointsBuffer: " << numPoints << "average " << newAverageNorm);
  }
  
  //pointsBufferAvg.averageNorm = newAverageNorm;
  
  if (newPoint.ring == 10)
  {
//     ROS_INFO_STREAM("updatePointsBuffer: " << numPoints << "average " << newAverageNorm);
  }
}


void interRingSubscriberCallback(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>& cloud_)
{
  ROS_INFO("interRingSubscriberCallback");
  pcl::PointCloud<velodyne_pointcloud::PointXYZIR> cloud = cloud_;
  pcl::PointCloud<velodyne_pointcloud::PointXYZIR> modifiedCloud;
  modifiedCloud.header = cloud_.header;
    
  // the list of obstacles in this call back
  std::vector<velodyne_pointcloud::PointXYZIR> currentObstaclesList; 

    
  // a map between a ring and the last point seen so far
  std::map<uint16_t,  velodyne_pointcloud::PointXYZIR> ringPointMap;
  
  std::vector< velodyne_pointcloud::PointXYZIR, Eigen::aligned_allocator <velodyne_pointcloud::PointXYZIR> > ::iterator iter = cloud.points.begin();
  
  std::map<uint16_t, std::vector<double > > distanceByPrevious;


  if(scan_nr==0) 
  {
     for(int i =0; i < NUM_RINGS; i++) 
     {
	g_medianFactorByRing[i] = 0;
     }
  }
  
  int order = 0;
  for (; iter != cloud.points.end(); iter++) {
    velodyne_pointcloud::PointXYZIR currentPoint = *iter;
    int currentIndex = currentPoint.ring;
    int previousIndex = currentIndex+1;
    if (currentIndex == NUM_RINGS-1)
      previousIndex = NUM_RINGS-2;
    
    if(ringPointMap.count(previousIndex) > 0 ) {
      velodyne_pointcloud::PointXYZIR prevPoint = ringPointMap[previousIndex];

      double currentTrueNorm = currentPoint.getVector4fMap().norm();
      double prevNorm = prevPoint.getVector4fMap().norm();
      double distanceFromPrevious = prevNorm-currentTrueNorm;
      double scaleFactor = prevNorm;//pow(prevNorm, 2);
      double expected_dist = g_medianFactorByRing[currentIndex]*scaleFactor;
      double difference = distanceFromPrevious - expected_dist;
      currentPoint.expected_dist = expected_dist;
      currentPoint.difference = difference;
      currentPoint.true_distance = currentTrueNorm;
  // std::cout << "***** diference: " << difference << "*****" << std::endl;
      if(currentIndex == NUM_RINGS-1) 
      {
	 difference*=-1;
      }
      if (difference>epsilon) 
      { // this is an obstacle;	
	currentPoint.obstacle = 10000.f;
	currentObstaclesList.push_back(currentPoint);
      }

      distanceByPrevious[currentIndex].push_back(distanceFromPrevious/scaleFactor);
  }
    ringPointMap[currentIndex] = currentPoint;
    if (currentPoint.ring == 15)
    {
      currentPoint.order = order++;
    }	
    //modifiedCloud.push_back(currentPoint);
  }
    std::vector<std::vector<velodyne_pointcloud::PointXYZIR> > clustering = clusterObstaclesSingleLink(currentObstaclesList, 7);
    
    for(int i =0; i<clustering.size(); i++) {
      std::vector<velodyne_pointcloud::PointXYZIR> currentCluster = clustering[i];
      for(int j=0; j < currentCluster.size(); j++) {
	modifiedCloud.push_back(currentCluster[j]);
      }
    }
      
   //if (scan_nr %100== 0){
     for (  std::map<uint16_t, std::vector<double > >::iterator iter = distanceByPrevious.begin(); iter != distanceByPrevious.end() ; ++iter)
     {
	int ring = (*iter).first;
	std::vector<double>& vec = (*iter).second;
	std::nth_element(vec.begin(), vec.begin()+(vec.size()/2), vec.end() );
	g_medianFactorByRing[ring] = vec[vec.size()/2];
	//std::cout << "*************ring, median: " << ring << ", " << vec[vec.size()/2];
     }
     //std::cout << std::endl;
   //}
   std_msgs::Float32 flt_msg;
   //flt_msg.data = (static_cast<float>(g_medianFactorByRing[14]));
   g_pub_median.publish(flt_msg);

   g_pub.publish(modifiedCloud);     
   scan_nr++;
}


int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "listener");

  
  
  ros::NodeHandle n;


// %Tag(SUBSCRIBER)%
  ros::Subscriber sub = n.subscribe("velodyne_points", 1000, interRingSubscriberCallback); // topic name: velodyne_points

  g_pub = n.advertise<pcl::PointCloud<velodyne_pointcloud::PointXYZIR> > ("output",1);
  g_pub_median = n.advertise<std_msgs::Float32> ("median",1);

// %EndTag(SUBSCRIBER)%

// %Tag(SPIN)%
  ros::spin();
// %EndTag(SPIN)%

  return 0;
}
// %EndTag(FULLTEXT)%
