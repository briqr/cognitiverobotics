
#include "utility.hpp"

#include <boost/concept_check.hpp>



Cluster::Cluster() {}
Cluster::~Cluster() {}
Cluster::Cluster(velodyne_pointcloud::PointXYZIR firstPoint){
addPoint(firstPoint);
}


void Cluster:: updateVariance(velodyne_pointcloud::PointXYZIR newPoint) {
	        int clusterSize = clusterPoints.size();
		if(clusterSize ==1) {
			mean.x = newPoint.x;
			mean.y = newPoint.y;
			mean.z = newPoint.z;
			return;
		}
 		Point3d mean_prev = mean;
		Point3d S_prev = S;
		mean.x = mean_prev.x + (newPoint.x - mean_prev.x)/clusterSize;
		mean.y = mean_prev.y + (newPoint.y - mean_prev.y)/clusterSize;
		mean.z = mean_prev.z + (newPoint.z - mean_prev.z)/clusterSize;
		S.x = S_prev.x + (newPoint.x - mean_prev.x) * (newPoint.x - mean.x);
		S.y = S_prev.y + (newPoint.y - mean_prev.y) * (newPoint.y - mean.y);
		S.z = S_prev.z + (newPoint.z - mean_prev.z) * (newPoint.z - mean.z);
		variance = (S.x+S.y+S.z)/(clusterSize-1);
	}
	
void Cluster:: addPoint(velodyne_pointcloud::PointXYZIR newPoint) {
  clusterPoints.push_back(newPoint);
  centroid.x = newPoint.x;
  centroid.y = newPoint.y;
  centroid.z = newPoint.z;
  updateVariance(newPoint);
}
void Cluster:: mergeCluster(Cluster& toMergeC) {
 
   // merge the clusters
    std::vector<velodyne_pointcloud::PointXYZIR> mergedClusterPoints;
     mergedClusterPoints.insert(mergedClusterPoints.end(), clusterPoints.begin(), clusterPoints.end());
     mergedClusterPoints.insert(mergedClusterPoints.end(), toMergeC.getClusterPoints().begin(), toMergeC.getClusterPoints().end());
     clusterPoints = mergedClusterPoints;
     toMergeC.clearCluster();
}

void Cluster:: clearCluster() {
  std::vector<velodyne_pointcloud::PointXYZIR>().swap(clusterPoints);
  closestDistance = std::numeric_limits<double>::infinity();
}