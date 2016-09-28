// %Tag(FULLTEXT)%
#include "ros/ros.h"
#include <ros/package.h>

#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

#include <pcl_ros/point_cloud.h>
#include <pcl/common/distances.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/conversions.h>
#include "../include/cognitive_robotics/point_type.h"

#include <boost/circular_buffer.hpp>

#include <cognitive_robotics/point_type.h>
#include "utility.hpp"

#include <visualization_msgs/Marker.h>


ros::Publisher g_pub;
ros::Publisher g_pub_median;
ros::Publisher g_pub_cluster_marker;
//std::vector <std::map<uint16_t,std::vector <std::vector<velodyne_pointcloud::PointXYZIR > > > > obstaclesList;
// the angle between 2 consecutive laser diodes
const int NUM_RINGS = 16;
const int WINDOW_SIZE = 500;
double diodeAngle = 2;
double epsilon = 0.15;
int scan_nr = 0;
int clusteringSize;
ClusterDescriptorVector g_clusters;
double clusterRadiusThreshold; // the threshold of the difference from a new point to the previous max raddius cluster, used to determine if we should start a new cluster (a new obstacle appeared)
double clusterVarianceThreshold;
// std::vector<double> g_clusterRadiuses; // the maximum distance of any point in the cluster to its center
std::map<std::string, double> g_paramsMap; 
std::map<uint16_t, double > g_medianFactorByRing;
size_t g_lastMarkerCount = 0;
pcl::PointCloud<velodyne_pointcloud::PointXYZIR> g_cloud ;


int ClusterDescriptor::last_id = 0;


void initParams(std::map<std::string, double > &paramsMap) {
  ros::NodeHandle nh;
  
  std::string path = ros::package::getPath("cognitive_robotics") + "/params.txt";
  
  std::string line;
  std::ifstream paramsFile (path); 
  if (paramsFile.is_open()){
    while ( getline (paramsFile,line) ){
      std::istringstream is_line(line);
      std::string key;
      if( std::getline(is_line, key, '=') ){
	std::string value;
	if( std::getline(is_line, value) ) {
	  //std:: cout << "******* value string : " << value;
	  paramsMap[key] = atof(value.c_str());
	}
    }
  }
  paramsFile.close();
 }
 else
 {
   ROS_ERROR_STREAM("could not open file" << path);
   exit(-1);
 }
}

void detectObstacles( pcl::PointCloud<velodyne_pointcloud::PointXYZIR>& cloud, std::vector<velodyne_pointcloud::PointXYZIR>& currentObstaclesList, std::map<uint16_t, double> &medianFactorByRing, pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &modifiedCloud, std::map<uint16_t, std::vector<double > > &distanceByPrevious) {
  int order = 0;
      // a map between a ring and the last point seen so far
  std::map<uint16_t,  velodyne_pointcloud::PointXYZIR> ringPointMap;

  std::vector< velodyne_pointcloud::PointXYZIR, Eigen::aligned_allocator <velodyne_pointcloud::PointXYZIR> > ::iterator iter = cloud.points.begin();

    // at first detect the obstacle points
  for (; iter != cloud.points.end(); iter++) {
        velodyne_pointcloud::PointXYZIR currentPoint = *iter;
        int currentIndex = currentPoint.ring;
	if(currentIndex > 7)
	  continue;
        int previousIndex = currentIndex-1;
        if (currentIndex == 0)
            previousIndex = 1;

	
	  
        if(ringPointMap.count(previousIndex) > 0 ) {
            velodyne_pointcloud::PointXYZIR prevPoint = ringPointMap[previousIndex];
            double prevNorm = prevPoint.getVector4fMap().norm();

	    
// 	    int ring_angle = 75+previousIndex*2;
// 	    float angle_to_next = (180-90-ring_angle)*M_PI/180;
// 	    float expected_dist = prevPoint.getVector4fMap().norm() / sin(M_PI-(M_PI-angle_to_next)-(2*M_PI/180)) * sin(M_PI-angle_to_next);
	
	    
	    float height = 0.95f;
	    float delta_angle = (2*M_PI/180);
	    float r_delta = height / sin( asin ( height / prevNorm ) - delta_angle ) - prevNorm;
	    
	    float expected_dist = prevNorm + r_delta;
	    
            double currentTrueNorm = currentPoint.getVector4fMap().norm();
            double distanceFromPrevious = prevNorm-currentTrueNorm;
            double scaleFactor = prevNorm;//pow(prevNorm, 2);
//             double expected_dist = g_medianFactorByRing[currentIndex]*scaleFactor;
//             double difference = distanceFromPrevious - expected_dist;
            double difference = (expected_dist - currentTrueNorm) / (prevNorm*prevNorm) ;
            currentPoint.expected_dist = expected_dist;
            currentPoint.difference = difference ;
            currentPoint.true_distance = currentTrueNorm;
            if(currentIndex == 0){
                difference*=-1;
            }
            if ( difference>0.008) { // this is an obstacle;
                currentPoint.obstacle = 10000.f;
                currentObstaclesList.push_back(currentPoint);
//             }
//             else {
		
	    }
	      modifiedCloud.push_back(currentPoint);

            distanceByPrevious[currentIndex].push_back(distanceFromPrevious/scaleFactor);
        }
        ringPointMap[currentIndex] = currentPoint;
        if (currentPoint.ring == 15)
        {
            currentPoint.order = order++;
        }
    }
}


 void clusterObstacles(const std::vector<velodyne_pointcloud::PointXYZIR>& obstacles, std::vector<std::vector<velodyne_pointcloud::PointXYZIR> > &currentClustering,  ClusterDescriptorVector& centers) {
    pcl::StopWatch timer;
    int initClusterSize = obstacles.size();//(int)g_paramsMap["initClusterSize"];
    currentClustering.reserve(initClusterSize);
    int numClusters = initClusterSize;
    int k = 100;
    int* minDistanceIndices = new int[initClusterSize];
    double* minDistanceValues = new double[initClusterSize];
    double* clusterInternalDistance = new double [initClusterSize];
    double** clusterDistances = new double*[initClusterSize];
    Point3d* variances = new Point3d[initClusterSize]; // the varaince of the point of a cluster
    
    for(int i = 0; i < initClusterSize; ++i) {
        clusterDistances[i] = new double[initClusterSize];
    }
    centers.erase(centers.begin(), centers.end());
    for(int i =0; i < initClusterSize; i++) { // the initial cluster contains each point from the dataset as a cluster
        //std:: cout<< "pushing back: " << obstacles[i].x<<",";
      std::vector<velodyne_pointcloud::PointXYZIR> newVec;  
      newVec.push_back(obstacles[i]);
      currentClustering.push_back(newVec);
      centers.push_back(ClusterDescriptor(Point3d(obstacles[i].x, obstacles[i].y, obstacles[i].z), 0.0));
      clusterInternalDistance[i]=0;
    }
    //std::cout<< "******beginning clustering size: "<< currentClustering.size();
    int minIndexi, minIndexj;
//     std::cout << "reached here!!!!!!!!!" <<std::endl;
    for(int i = 0; i < initClusterSize; ++i) {
        minDistanceValues[i] =  std::numeric_limits<double>::infinity();
    }
 
    double minDistance =-1;
    do {
        if(numClusters == initClusterSize) {// this is the first iteration
            for(int i =0; i<numClusters; i++) {
                clusterDistances[i][i] = std::numeric_limits<double>::infinity();
                for (int j=i+1; j<numClusters ; j++) {
                    double currentDistance= distanceBetweenClusters(currentClustering[i], currentClustering[j]);
                    clusterDistances[i][j] = currentDistance;
                    clusterDistances[j][i] = currentDistance;
                    if(currentDistance < minDistanceValues[i]) {
                        minDistanceIndices[i]=j;
                        minDistanceValues[i]=currentDistance;	
                    }
                    if(currentDistance < minDistanceValues[j]) {  // actually not needed, just for consistency sake
                        minDistanceIndices[j]=i;
                        minDistanceValues[j]=currentDistance;	
                    }
                }
            }
        }
         
        //find the minimum distance, but also make sure that it's not larger than the inernal distance of the clusters that we want to merge by a big margin, otherwise, look for the next min distance
      bool distanceExceeded = false;
      do {
        minDistance = std::numeric_limits<double>::infinity();
        for(int i =0; i<initClusterSize; i++) {
            double currentDistance= minDistanceValues[i];
            if(currentDistance < minDistance) {
                minDistance = currentDistance;
                minIndexi = i;
                minIndexj = minDistanceIndices[i];
            }
        }
        if(numClusters > initClusterSize/2)
	    break;
	  double lastMinDistance = minDistance;
	  // nothing left to merge, we can stop the merging
	if(std::isinf(minDistance)) {
	  break;
	}
	// This condition checks whether we should merge these 2 clusters with the min distance based on the internal distance of clusteri and clusterj  
	  if(fabs(lastMinDistance - clusterInternalDistance[minIndexi]) > g_paramsMap["clusterDistanceThreshold"]|| fabs(lastMinDistance - clusterInternalDistance[minIndexj]) > g_paramsMap["clusterDistanceThreshold"] || clusterInternalDistance[minIndexi] > g_paramsMap["maxInternalDistance"] || clusterInternalDistance[minIndexj] > g_paramsMap["maxInternalDistance"] ) { // find the next nearest distance
	    distanceExceeded = true;
	    clusterDistances[minIndexi][minIndexj] = std::numeric_limits<double>::infinity(); // we don't want to consider these 2 clusters for merging anymore
	    clusterDistances[minIndexj][minIndexi] = std::numeric_limits<double>::infinity();
	    minDistanceValues[minIndexi] = std::numeric_limits<double>::infinity();
	    minDistance = std::numeric_limits<double>::infinity();
	    for(int i =0; i<initClusterSize; i++) { // find the next min distance to cluster i
	      if(i != minIndexj) {
		  if(clusterDistances[minIndexi][i] < minDistance) {
		      minDistance = clusterDistances[minIndexi][i];
		      minDistanceValues[minIndexi] = minDistance;
		      minDistanceIndices[minIndexi] = i;
		  }
	      }
	    }
	 }
	 else {
	  distanceExceeded = false;  
	 }

      }while(distanceExceeded);
        
	//update the affected distances as a result of merging
	double currentMinDistance = std::numeric_limits<double>::infinity();
        for(int i =0; i<initClusterSize; i++) {
            if(i==minIndexi) {
                continue;
            }
            if(i != minIndexj) {
                clusterDistances[minIndexi][i] = std::min(clusterDistances[minIndexi][i], clusterDistances[minIndexj][i]);
                clusterDistances[i][minIndexi] =  clusterDistances[minIndexi][i];
                if(clusterDistances[minIndexi][i] < currentMinDistance) {
                    currentMinDistance = clusterDistances[minIndexi][i];
                    minDistanceValues[minIndexi] = currentMinDistance;
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
    
	updateCentre(centers[minIndexi].center, currentClustering[minIndexi].size(),currentClustering[minIndexj]);
	centers[minIndexj].center = Point3d(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
	  
	
        // merge the clusters
        std::vector<velodyne_pointcloud::PointXYZIR> mergedCluster;
        mergedCluster.insert(mergedCluster.end(), currentClustering[minIndexi].begin(), currentClustering[minIndexi].end());
        mergedCluster.insert(mergedCluster.end(), currentClustering[minIndexj].begin(), currentClustering[minIndexj].end());
	  
//         std::cout << "coutmergedCluster size:" << mergedCluster.size() <<  std::endl;
        currentClustering[minIndexi].swap(mergedCluster);
        currentClustering[minIndexj].erase(currentClustering[minIndexj].begin(),  currentClustering[minIndexj].end());

	updateVariance(currentClustering[minIndexi], variances[minIndexi]);
	variances[minIndexj] = Point3d(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
	updateInternalDistance(currentClustering[minIndexi], clusterInternalDistance[minIndexi]);
	  
        clusterInternalDistance[minIndexj] = std::numeric_limits<double>::infinity();
	--numClusters;
       
    } while(!std::isinf(minDistance));
    ROS_INFO_STREAM("-----final cluster size: " << numClusters << ", init size: " << initClusterSize);
    // compute the cluster radiuses
     
//     std::cout << "after merge: " << currentClustering.size() <<std::endl;
    for(int i = 0; i < initClusterSize; ++i) {
        delete [] clusterDistances[i];
    }
    delete []clusterDistances;

    delete [] minDistanceIndices;
    delete [] minDistanceValues;
    delete[] clusterInternalDistance;
    delete [] variances;
    // assign id to clusters
//     std::cout<< "total num points: (" << initClusterSize <<"), ";
    
    cleanClusters(currentClustering, centers);
     
    //colorCluster(currentClustering, centers);
    
    computeClusterRadiuses( centers, currentClustering);
    
    //std::cout<< "******end clustering size: "<< currentClustering[0].size() << std::endl;
    ROS_INFO_STREAM("time:" << timer.getTime());
}

// this function introduces a motion model into kmeans clustering
 void clusterUsingExistingClustering(const std::vector<velodyne_pointcloud::PointXYZIR>& obstacles, std::vector<std::vector<velodyne_pointcloud::PointXYZIR> >& clustering,  ClusterDescriptorVector& centers, const ros::Time& timeStamp ) {
   pcl::StopWatch timer;
   std::vector<velodyne_pointcloud::PointXYZIR> unassingedObstacles;
   std::vector<Point3d> newCenters;
   int *pointAssignments  = new int[obstacles.size()]; // the index of the cluster a point is assigned to
   
   int k = centers.size();
    
   std::vector<bool> clusterHasAssignment;
   
   updateCentersBasedOnVelocity(centers, timeStamp );

   updateLastCenters(centers);
   
    bool changeOccured = false; // indicates if a change in assignment occured
    int kMeansIterNum = 0;
    do{
      changeOccured = false;
      for(int i = 0; i < obstacles.size(); ++i) {
	double minDistance =  std::numeric_limits<double>::infinity();
	int clusterNum;
	for(int j =0; j<centers.size(); j++) {
	  double currentDistance = distance2Points(centers[j].center, obstacles[i]);
	  if(currentDistance < minDistance) {
	    minDistance = currentDistance;
	    clusterNum = j;
	  }
      }
        double distanceFromCenter = minDistance;
      if(distanceFromCenter > centers[clusterNum].radius+g_paramsMap["clusterRadiusThreshold"]) { // probably a new obstacle appeared, start a new cluster for it
	pointAssignments[i] = -1;
	  continue;
      }
      int prevAssignment = pointAssignments[i];
      if(prevAssignment != clusterNum) {
	changeOccured = true;
	pointAssignments[i] = clusterNum;
      }
    }
    updateClusterCentersRadiuses(centers, pointAssignments, obstacles, timeStamp, clusterHasAssignment);
    ++kMeansIterNum;
  }
    while(changeOccured);    
    for(int i =0; i<k; i++) { // clear the outdated clustering
      std::vector<velodyne_pointcloud::PointXYZIR> newVec;
      clustering.push_back(newVec);
    }
    for(int i=0; i< obstacles.size(); i++) {
      int clusterAssignment = pointAssignments[i];
      if(clusterAssignment < 0)
	continue;
      clustering[clusterAssignment].push_back(obstacles[i]);
    }
    // remove empty clusters
  std::vector<std::vector<velodyne_pointcloud::PointXYZIR> > newClustering;
  for(int i = 0 ; i < obstacles.size(); i++) {
    if(pointAssignments[i]==-1)  {
      unassingedObstacles.push_back(obstacles[i]);
    }
  }
  if( unassingedObstacles.size()> 0) {
    ClusterDescriptorVector newClustersCenters;
   ROS_INFO_STREAM("****---Creating clusters for emerging obstacles");
   clusterObstacles(unassingedObstacles, newClustering, newClustersCenters);
   ROS_INFO_STREAM("****--Done creating clusters for emerging obstacles");
//    clustering.insert(clustering.end(), newClustering.begin(), newClustering.end());
//    centers.insert(centers.end(), newClustersCenters.begin(),newClustersCenters.end());
//    std::vector<bool> newClusterHasAssignments;
//    newClusterHasAssignments.resize(newClustersCenters.size());
//    clusterHasAssignment.insert(clusterHasAssignment.begin(), newClusterHasAssignments.begin(), newClusterHasAssignments.end());
  }

  updateTimeStamps(centers, timeStamp, clusterHasAssignment );
  updateState(centers, timeStamp, clusterHasAssignment );

  
  cleanClusters(clustering, centers, clusterHasAssignment);
  ROS_INFO_STREAM("---number kmeans iterations " << kMeansIterNum <<", number of clusters: " << clustering.size() << ", obstacles number: " << obstacles.size());
  /*if(clustering.size() < k)
    ROS_INFO_STREAM("******************Number of clusters reduced from: " << k <<" to: " << clustering.size());*/
  colorCluster(clustering, centers);
  delete pointAssignments;
  ROS_INFO_STREAM("time:" << timer.getTime());
 }

void interRingSubscriberCallback(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>& cloud_)
{
    ROS_INFO("interRingSubscriberCallback");
//     if(scan_nr==0)
//     {
//       g_cloud = cloud_;
//     }
//     
    ClusterDescriptor::last_id = 0;

    
    pcl::PointCloud<velodyne_pointcloud::PointXYZIR> cloud = cloud_;
//     pcl::PointCloud<velodyne_pointcloud::PointXYZIR> cloud = g_cloud;
    pcl::PointCloud<velodyne_pointcloud::PointXYZIR> modifiedCloud;
    modifiedCloud.header = cloud_.header;
    std::map<uint16_t, std::vector<double > > distanceByPrevious;

    // the list of obstacles in this call back
    std::vector<velodyne_pointcloud::PointXYZIR> currentObstaclesList;

    if(scan_nr==0)
    {
        initParams(g_paramsMap);
        for(int i =0; i < NUM_RINGS; i++){
            g_medianFactorByRing[i] = 0;
        }
    }
    detectObstacles(cloud, currentObstaclesList, g_medianFactorByRing, modifiedCloud, distanceByPrevious);
    std::vector<std::vector<velodyne_pointcloud::PointXYZIR> > clustering;
    if(scan_nr ==0) {
      // for the initial cluster we use heirarichal clustering, this gives an idea about what the number of cluster k should be
      clusterObstacles(currentObstaclesList, clustering, g_clusters );
    }
    else {// use the previous data and clustering to determine the clustering of new scans with possibly not a very different clustering
      clusterUsingExistingClustering(currentObstaclesList, clustering, g_clusters, pcl_conversions::fromPCL(cloud_.header.stamp) );
    }
    
 
    for(int i =0; i<clustering.size(); i++) {
        std::vector<velodyne_pointcloud::PointXYZIR> currentCluster = clustering[i];
        for(int j=0; j < currentCluster.size(); j++) {
            modifiedCloud.push_back(currentCluster[j]);
        }
    }

    for (std::map<uint16_t, std::vector<double > >::iterator iter = distanceByPrevious.begin(); iter != distanceByPrevious.end() ; ++iter){
        int ring = (*iter).first;
        std::vector<double>& vec = (*iter).second;
        std::nth_element(vec.begin(), vec.begin()+(vec.size()/2), vec.end() );
        g_medianFactorByRing[ring] = vec[vec.size()/2];
    }

    //std_msgs::Float32 flt_msg;
    //flt_msg.data = (static_cast<float>(g_medianFactorByRing[14]));
    //g_pub_median.publish(flt_msg);


    publishClusterMarker(g_pub_cluster_marker, g_clusters, g_lastMarkerCount, pcl_conversions::fromPCL(cloud_.header.stamp));
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
    g_pub_cluster_marker = n.advertise<visualization_msgs::Marker>( "cluster_marker", 0 );	
// %EndTag(SUBSCRIBER)%

// %Tag(SPIN)%
    ros::spin();
// %EndTag(SPIN)%

    return 0;
}
// %EndTag(FULLTEXT)%
