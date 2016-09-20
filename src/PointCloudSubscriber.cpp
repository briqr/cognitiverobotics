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
#include "utility.hpp"

#include <visualization_msgs/Marker.h>


ros::Publisher g_pub;
ros::Publisher g_pub_median;
ros::Publisher g_pub_cluster_marker;
const int NUM_RINGS = 16;
const int WINDOW_SIZE = 500;
std::vector <std::map<uint16_t,std::vector <std::vector<velodyne_pointcloud::PointXYZIR > > > > obstaclesList;
// the angle between 2 consecutive laser diodes
double diodeAngle = 2;
double epsilon = 0.05;
int scan_nr = 0;
int clusteringSize;
std::vector<std::pair<Point3d, double>> g_clusters;
double clusterRadiusThreshold = 2; // the threshold of the difference from a new point to the previous max raddius cluster, used to determine if we should start a new cluster (a new obstacle appeared)
double clusterVarianceThreshold = 5;
// std::vector<double> g_clusterRadiuses; // the maximum distance of any point in the cluster to its center
std::map<uint16_t, double > g_medianFactorByRing;
size_t g_lastMarkerCount = 0;

 void clusterObstacles(const std::vector<velodyne_pointcloud::PointXYZIR>& obstacles, std::vector<std::vector<velodyne_pointcloud::PointXYZIR> > &currentClustering,  std::vector<std::pair<Point3d,double>>& centers) {
    pcl::StopWatch timer;
    int initClusterSize = obstacles.size();
    currentClustering.reserve(initClusterSize);
    int numClusters = initClusterSize;
    int k = 100;
    int* minDistanceIndices = new int[initClusterSize];
    double* minDistanceValues = new double[initClusterSize];
    double* variances = new double[initClusterSize];
    double** clusterDistances = new double*[initClusterSize];
    for(int i = 0; i < initClusterSize; ++i) {
        clusterDistances[i] = new double[initClusterSize];
    }
    centers.erase(centers.begin(), centers.end());
    for(int i =0; i < initClusterSize; i++) { // the initial cluster contains each point from the dataset as a cluster
        //std:: cout<< "pushing back: " << obstacles[i].x<<",";
      std::vector<velodyne_pointcloud::PointXYZIR> newVec;  
      newVec.push_back(obstacles[i]);
      currentClustering.push_back(newVec);
      centers.push_back(std::make_pair<Point3d, double>(Point3d(obstacles[i].x, obstacles[i].y, obstacles[i].z), 0.0));
    }
    //std::cout<< "******beginning clustering size: "<< currentClustering.size();
    int minIndexi, minIndexj;
    //std::cout << "reached here!!!!!!!!!" <<std::endl;
    for(int i = 0; i < initClusterSize; ++i) {
        minDistanceValues[i] =  std::numeric_limits<double>::infinity();
        variances[i] =  -1;
    }
    double maxVariance = -1;


    //while(numClusters > k) { // merge cluster based on the closest distance
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
    
	updateCentre(centers[minIndexi].first, currentClustering[minIndexi].size(),currentClustering[minIndexj]);
	centers[minIndexj].first = Point3d(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
	  
	
        // merge the clusters
        std::vector<velodyne_pointcloud::PointXYZIR> mergedCluster;
        mergedCluster.insert(mergedCluster.end(), currentClustering[minIndexi].begin(), currentClustering[minIndexi].end());
        mergedCluster.insert(mergedCluster.end(), currentClustering[minIndexj].begin(), currentClustering[minIndexj].end());
	  
        //std::cout << "coutmergedCluster size:" << mergedCluster.size() <<  std::endl;
        currentClustering[minIndexi] = mergedCluster;
        currentClustering[minIndexj].erase(currentClustering[minIndexj].begin(),  currentClustering[minIndexj].end());
        
        --numClusters;
        if(maxVariance > -1) {

            variances[minIndexi] = updateVariance(currentClustering[minIndexi]);
            variances[minIndexj] = std::numeric_limits<double>::infinity();
            if(variances[minIndexi]>maxVariance) {
                maxVariance = variances[minIndexi];
            }
        }
        else if(numClusters < initClusterSize/2) {
            for (int i=0; i<numClusters ; i++) {
                if(currentClustering[i].size() <2) {
                    continue;
                }
                if(variances[i] < std::numeric_limits<double>::infinity()) {

                    variances[i] = updateVariance(currentClustering[i]);
                    if(variances[i]>maxVariance) {
                        maxVariance = variances[i];
                    }
                }
                //std::cout <<"max variance" <<  maxVariance << ", ";
            }
        }
        //double diff = maxVariance-prevMaxVariance - prevDiff;
    } while(maxVariance < clusterVarianceThreshold);
    std:: cout << "-----final cluster size" << numClusters << ", init size" << initClusterSize;
    // compute the cluster radiuses
     
    //std::cout << "after merge: " << currentClustering.size() <<std::endl;
    for(int i = 0; i < initClusterSize; ++i) {
        delete [] clusterDistances[i];
    }
    delete []clusterDistances;

    delete [] minDistanceIndices;
    delete [] minDistanceValues;
    delete[] variances;
    // assign id to clusters
    //std::cout<< "total num points: (" << initClusterSize <<"), ";
    
    cleanClusters(currentClustering, centers);
     
    colorCluster(currentClustering);
    
    computeClusterRadiuses( centers, currentClustering);
    
    //std::cout<< "******end clustering size: "<< currentClustering[0].size() << std::endl;
    ROS_INFO_STREAM("time:" << timer.getTime());
}

// this function introduces a motion model into kmeans clustering
 void clusterUsingExistingClustering(const std::vector<velodyne_pointcloud::PointXYZIR>& obstacles, std::vector<std::vector<velodyne_pointcloud::PointXYZIR> >& clustering,  std::vector<std::pair<Point3d,double>>& centers ) {
   pcl::StopWatch timer;
   std::vector<Point3d> newCenters;
   int *pointAssignments  = new int[obstacles.size()]; // the index of the cluster a point is assigned to
   int k = centers.size();
   int *clusterSizes = new int[k];
    for(int j =0; j<k; j++) {
      std::vector<velodyne_pointcloud::PointXYZIR> newVec;
	clustering.push_back(newVec);
	clusterSizes[j] = 0;
    }
    for(int i = 0; i < obstacles.size(); ++i) {
      double minDistance =  std::numeric_limits<double>::max();
      int clusterNum;
      for(int j =0; j<k; j++) {
	double currentDistance = distance2Points(centers[j].first, obstacles[i]);
	if(currentDistance < minDistance) {
	  minDistance = currentDistance;
	  clusterNum = j;
	}
      }
      double distanceFromCenter = distance2Points(centers[clusterNum].first, obstacles[i]);
//       if(distanceFromCenter > centers[clusterNum].second+clusterRadiusThreshold) { // probably a new obstacle appeared
//    	std::cout << "new obstacle appeared: " << distanceFromCenter - centers[clusterNum].second << std::endl;
//       }
      /*else {
	std::cout << "NO!! new obstacle appeared" << std::endl;
      }*/
      pointAssignments[i] = clusterNum;
      ++clusterSizes[clusterNum];

      clustering[clusterNum].push_back(obstacles[i]);
      int newSize = clustering[clusterNum].size();
      updateExistingCentre(centers[clusterNum].first, newSize-1, obstacles[i]);
    }
    /*int sum = 0;
    int max = 0;
    for(int i =0; i < k; i++) {
      sum+=clustering[i].size();
      if(clustering[i].size() > clustering[max].size())
	max = i;
    }*/
    bool changeOccured = false; // indicates if a change in assignment occured
    int kMeansIterNum = 0;
    do{
      changeOccured = false;
      for(int i = 0; i < obstacles.size(); ++i) {
      double minDistance =  std::numeric_limits<double>::infinity();
      int clusterNum;
      for(int j =0; j<centers.size(); j++) {
	double currentDistance = distance2Points(centers[j].first, obstacles[i]);
	if(currentDistance < minDistance) {
	  minDistance = currentDistance;
	  clusterNum = j;
	}
      }
      int prevAssignment = pointAssignments[i];
      if(prevAssignment != clusterNum) {
	changeOccured = true;
	--clusterSizes[prevAssignment];
	++clusterSizes[clusterNum];
	pointAssignments[i] = clusterNum;
        int newSize = clusterSizes[clusterNum];
        updateExistingCentre(centers[clusterNum].first, newSize-1, obstacles[i]);
      }
    }
    updateClusterRadiuses(centers, pointAssignments, obstacles);
    ++kMeansIterNum;
  }
    while(changeOccured);
    ROS_INFO_STREAM("---number kmeans iterations " << kMeansIterNum);
    if(kMeansIterNum == 1) { // meanning the clustering assignment from before kmeans has changed
      int clusterSize = clustering.size();
      clustering.erase(clustering.begin(), clustering.end());
      for(int i =0; i<clusterSize; i++) { // clear the outdated clustering
	std::vector<velodyne_pointcloud::PointXYZIR> newVec;
	clustering.push_back(newVec);
      }
      for(int i=0; i< obstacles.size(); i++) {
	int clusterAssignment = pointAssignments[i];
	clustering[clusterAssignment].push_back(obstacles[i]);
      }
    }
    // remove empty clusters
  cleanClusters(clustering, centers);
  colorCluster(clustering);
 delete pointAssignments;
 delete clusterSizes;
 ROS_INFO_STREAM("time:" << timer.getTime());
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
    // at first detect the obstacle points
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
            if(currentIndex == NUM_RINGS-1){
                difference*=-1;
            }
            if (difference>epsilon) { // this is an obstacle;
                currentPoint.obstacle = 10000.f;
                currentObstaclesList.push_back(currentPoint);
            }
            else {
		modifiedCloud.push_back(currentPoint);
	    }
	      

            distanceByPrevious[currentIndex].push_back(distanceFromPrevious/scaleFactor);
        }
        ringPointMap[currentIndex] = currentPoint;
        if (currentPoint.ring == 15)
        {
            currentPoint.order = order++;
        }
    }
    std::vector<std::vector<velodyne_pointcloud::PointXYZIR> > clustering;
    if( scan_nr ==0|| scan_nr ==1) {
      // for the initial cluster we use heirarichal clustering, this gives an idea about what the number of cluster k should be
      clusterObstacles(currentObstaclesList, clustering, g_clusters );
    }
    else {// use the previous data and clustering to determine the clustering of new scans with possibly not a very different clustering
      clusterUsingExistingClustering(currentObstaclesList, clustering, g_clusters );
    }
    
 
    for(int i =0; i<clustering.size(); i++) {
        std::vector<velodyne_pointcloud::PointXYZIR> currentCluster = clustering[i];
        for(int j=0; j < currentCluster.size(); j++) {
            modifiedCloud.push_back(currentCluster[j]);
        }
    }

    for (  std::map<uint16_t, std::vector<double > >::iterator iter = distanceByPrevious.begin(); iter != distanceByPrevious.end() ; ++iter)
    {
        int ring = (*iter).first;
        std::vector<double>& vec = (*iter).second;
        std::nth_element(vec.begin(), vec.begin()+(vec.size()/2), vec.end() );
        g_medianFactorByRing[ring] = vec[vec.size()/2];

    }

    //std_msgs::Float32 flt_msg;
    //flt_msg.data = (static_cast<float>(g_medianFactorByRing[14]));
    //g_pub_median.publish(flt_msg);

    g_pub.publish(modifiedCloud);

    publishClusterMarker(g_pub_cluster_marker, g_clusters, g_lastMarkerCount  );
    
    scan_nr++;
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "listener");



    ros::NodeHandle n;


// %Tag(SUBSCRIBER)%
    ros::Subscriber sub = n.subscribe("velodyne_points", 1, interRingSubscriberCallback); // topic name: velodyne_points

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
