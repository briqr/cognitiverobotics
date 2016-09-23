#ifndef __UTILITY_H
// #define __UTILITY_H

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
#include <visualization_msgs/Marker.h>


struct Point3d {
  Point3d() : x(0), y(0), z(0){}
  Point3d(double x_, double y_, double z_) : x(x_), y(y_), z(z_){}
	double x;
	double y;
	double z;
	
	Point3d& operator+(const Point3d& rhs){ 
            x += rhs.x;
            y += rhs.y;
	    z += rhs.z;
            return *this;
    }
    Point3d& operator/(const double& a){ 
            x /= a;
            y /= a;
	    z /= a;
            return *this;
    }
};


struct ClusterDescriptor{
  ClusterDescriptor() :  ClusterDescriptor(Point3d(), 0.0 ){};

  ClusterDescriptor(Point3d _center, double _radius) : center(_center), radius(_radius)
  {
    id = last_id++;

    
  };
  
  int id;
  Point3d center;
  double radius;
  
  static int last_id;

};


typedef std::vector <ClusterDescriptor> ClusterDescriptorVector;


struct Variance {
 Point3d mean; // the cluster estimated mean per dimension
 Point3d S; // sum of squares of the difference between the value and the mean per dimension
 double variance;
};


class Cluster {
private:
	Point3d centroid;
	std::vector<velodyne_pointcloud::PointXYZIR> clusterPoints;
	double variance; // the inter variance of the cluster
	double closestDistance; //the closest distance to a cluster from the set of clusters
	int closestIndex; // the index of the closest cluster
	Point3d mean; // the cluster estimated mean per dimension
	Point3d S; // sum of squares of the difference between the value and the mean per dimension
	double radius; //the radius of the cluster, average some of distanances from the centre
	
	
	void updateVariance(velodyne_pointcloud::PointXYZIR newPoint);
public:
	Cluster(velodyne_pointcloud::PointXYZIR firstPoint);
	Cluster();
	~Cluster();
	 
	Point3d getCentroid() const {
	 return centroid;
	 }
	void addPoint(velodyne_pointcloud::PointXYZIR newPoint);
	 std::vector<velodyne_pointcloud::PointXYZIR>  getClusterPoints() {
	   return clusterPoints;
	 }
	double getVariance() {
	  return variance;
	}
	double getMinDistance() const {
	  return closestDistance;
	}
	int getMinIndex() const {
	  return closestIndex;
	}
	
	int getClusterSize() const {
	  return clusterPoints.size();
	}
	
	void mergeCluster(Cluster& toMergeC);
	
	void setMinDistance(double minDist) {
	  closestDistance = minDist;
	}
	
	void setMinIndex(int minIndex) {
	  closestIndex = minIndex;
	}
	
	void clearCluster() ;
	
};


static unsigned int hashColor( unsigned int hash_cluster_id )
{
  hash_cluster_id = ((hash_cluster_id >> 16) ^ hash_cluster_id) * 0x45d9f3b;
  hash_cluster_id = ((hash_cluster_id >> 16) ^ hash_cluster_id) * 0x45d9f3b;
  hash_cluster_id = (hash_cluster_id >> 16) ^ hash_cluster_id;
    
  return hash_cluster_id;
}

 static void getRainbowColor(float value, float& r, float& g, float& b)
{
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

static void updateInternalDistance(std::vector<velodyne_pointcloud::PointXYZIR> cluster, double& internalDistance) {
  int n = cluster.size();
  // there are n(n-1)/2 pairs in the cluster
  int scale = n*(n-1)/2;
  internalDistance = 0;
    for(int i =0; i<cluster.size(); i++) {
      for(int j=i+1;j<cluster.size(); j++) {
	internalDistance += pcl::euclideanDistance<velodyne_pointcloud::PointXYZIR>(cluster[i], cluster[j]);
      }
    }
    internalDistance/=scale;
}


static void updateInternalWeightedDistance(std::vector<velodyne_pointcloud::PointXYZIR> cluster, double& internalDistance, const Point3d& variance) {
  int n = cluster.size();
  // there are n(n-1)/2 pairs in the cluster
  int scale = n*(n-1)/2;
  internalDistance = 0;
    for(int i =0; i<cluster.size(); i++) {
      for(int j=i+1;j<cluster.size(); j++) {
	//internalDistance += sqrt(pow((cluster[i].x-cluster[j].x)/variance.x, 2) + pow((cluster[i].y-cluster[j].y)/variance.y, 2)+pow((cluster[i].z-cluster[j].z)/variance.z, 2));
// 	internalDistance += sqrt(pow((cluster[i].x-cluster[j].x)/variance.x, 2) + pow((cluster[i].y-cluster[j].y)/variance.y, 2));
      }
    }
    internalDistance/=scale;
}


static void updateVariance(std::vector<velodyne_pointcloud::PointXYZIR> cluster1, Point3d &variance){
  int n = cluster1.size();
  Point3d mean;
  for(int i =0; i<n; i++) {
  mean.x += cluster1[i].x;
  mean.y += cluster1[i].y;
  mean.z += cluster1[i].z;
 }
  
  mean.x /= n;
  mean.y /= n;
  mean.z /= n;
  variance = Point3d(0,0,0);
  for(int i =0; i<cluster1.size(); i++) {
    variance.x += pow(cluster1[i].x-mean.x, 2);
    variance.y += pow(cluster1[i].y-mean.y, 2);
    variance.z += pow(cluster1[i].z-mean.z, 2);
  }
  variance.x/=(n-1);
  variance.y/=(n-1);
  variance.z/=(n-1);
}
static double updateExistingCentre(Point3d& initialCentre, int initialSize, const velodyne_pointcloud::PointXYZIR &newPoint) {
    Point3d newCentre(initialCentre.x*initialSize, initialCentre.y*initialSize, initialCentre.z*initialSize);
     newCentre.x+=newPoint.x;
    newCentre.y+=newPoint.y;
    newCentre.z+=newPoint.z;
   newCentre.x/=initialSize+1;
   newCentre.y/=initialSize+1;
   newCentre.z/=initialSize+1;
   initialCentre = newCentre;

}

static double updateCentre(Point3d& initialCentre, int initialSize, const std::vector<velodyne_pointcloud::PointXYZIR>& clusterj) {
  Point3d newCentre(initialCentre.x*initialSize, initialCentre.y*initialSize, initialCentre.z*initialSize);
  for(int i =0; i < clusterj.size(); i++) {
    newCentre.x+=clusterj[i].x;
    newCentre.y+=clusterj[i].y;
    newCentre.z+=clusterj[i].z;
  }
   newCentre.x/=(clusterj.size()+initialSize);
   newCentre.y/=(clusterj.size()+initialSize);
   newCentre.z/=(clusterj.size()+initialSize);
   initialCentre = newCentre;
}
static double distanceBetweenClusters(const std::vector<velodyne_pointcloud::PointXYZIR>& clusteri, const std::vector<velodyne_pointcloud::PointXYZIR>& clusterj) {
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


static double distance2Points(const Point3d point1, const velodyne_pointcloud::PointXYZIR point2){
  double distance = pow(point1.x-point2.x, 2) + pow(point1.y-point2.y, 2) + pow(point1.z-point2.z, 2);
  return sqrt(distance);
}

static int generateRand(int min, int max){
  int rndVal = min + (std::rand() % (int)(max - min + 1));
  //std::cout << "rnd:"  << rndVal<<", ";
  return rndVal;
}

static void colorCluster(std::vector<std::vector<velodyne_pointcloud::PointXYZIR> > &currentClustering, const ClusterDescriptorVector& clusterCenters) {
  int clusterId = 1;
  //int random_number = generateRand(1, 1000);
  for(int i =0; i<currentClustering.size(); i++) {
        unsigned int hash_cluster_id = hashColor(clusterCenters[i].id);

        float c, r, g, b;
	c = ((hash_cluster_id)%255)/(double)255;

        //ROS_INFO_STREAM( "clusterid: " << clusterId << " " << c );
        getRainbowColor(c,r,g,b);
        for(int j=0; j<currentClustering[i].size(); j++) {
            (currentClustering[i])[j].cluster_id = clusterCenters[i].id;
            //uint32_t rgb = ((uint32_t)22 << 16 | (uint32_t)clusterId << 8 | (uint32_t)10);
            (currentClustering[i])[j].r = r*255;
            (currentClustering[i])[j].g = g*255;
            (currentClustering[i])[j].b = b*255;
        }
        ++clusterId;
// 	random_number = generateRand(1, 1000);
    }
}
// first time computation of the cluster radiuses
static void computeClusterRadiuses(ClusterDescriptorVector& clusterCenters, const std::vector<std::vector<velodyne_pointcloud::PointXYZIR> > & currentClustering) {
  for(int i =0; i< clusterCenters.size(); i++) {
      double maxRadius = -1;
      for(int j=0; j<currentClustering[i].size(); j++) {
	double currentRadius = distance2Points(clusterCenters[i].center, currentClustering[i][j]); 
	if(currentRadius > maxRadius) {
	  maxRadius = currentRadius;
	}
      }
      clusterCenters[i].radius = maxRadius;
   }
}

//update the cluster radiuses dueing the kmean iterations
static void updateClusterCentersRadiuses(ClusterDescriptorVector& clusterCenters, const int *pointAssignments, const std::vector<velodyne_pointcloud::PointXYZIR>& obstacles) {
   int *clusterSizes  = new int[clusterCenters.size()]; 
  int i = 0;
  for(auto& e : clusterCenters ){
     Point3d newCentre;
     e.center = newCentre;
    e.radius = 0.0;
    clusterSizes[i++] = 0;
  }
  // first update the centers
  for(i =0; i< obstacles.size(); i++) {
   int assignment = pointAssignments[i];
   if(assignment < 0)
     continue;
   updateExistingCentre(clusterCenters[assignment].center, clusterSizes[assignment], obstacles[i]);
   ++clusterSizes[assignment];
  }
  // now update the radiuses
  for(i =0; i< obstacles.size(); i++) {
    int assignment = pointAssignments[i];
    if(assignment < 0)
     continue;
    double currentRadius = distance2Points(clusterCenters[assignment].center, obstacles[i]); 
    if(clusterCenters[assignment].radius < currentRadius) {
      clusterCenters[assignment].radius = currentRadius;
    }
  }
  delete clusterSizes;
}

static void cleanClusters(std::vector<std::vector<velodyne_pointcloud::PointXYZIR> >& clustering, ClusterDescriptorVector& centers) {
  std::vector<std::vector<velodyne_pointcloud::PointXYZIR> > tmpClustering;
  ClusterDescriptorVector  tmpCenters;
  for(int i =0; i<clustering.size(); i++) {
    if(clustering[i].size()>1) {
    tmpClustering.push_back(clustering[i]); 
    tmpCenters.push_back(centers[i]); 
    }
   }
   clustering.swap(tmpClustering);
   centers.swap(tmpCenters);
}

static void publishClusterMarker(ros::Publisher publisher, const ClusterDescriptorVector clusterCenters, size_t& last_marker_count, ros::Time stamp ){
  
  for (size_t i = 0; i < last_marker_count; i++) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "velodyne";
    marker.header.stamp = stamp;
    marker.ns = "cluster";
    marker.id = i;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::DELETE;
 
    publisher.publish( marker );
  }
  
  for (size_t i = 0; i < clusterCenters.size(); i++) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "velodyne";
    marker.header.stamp = stamp;
    marker.ns = "cluster";
    marker.id = i;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = clusterCenters[i].center.x;
    marker.pose.position.y = clusterCenters[i].center.y;
    marker.pose.position.z = clusterCenters[i].center.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = clusterCenters[i].radius;
    marker.scale.y = clusterCenters[i].radius;
    marker.scale.z = clusterCenters[i].radius;
    
    //ROS_INFO_STREAM("radius: " << clusterRadiuses[i]);
    
    unsigned int hash_cluster_id = hashColor(clusterCenters[i].id);
      
    float c, r, g, b;
    c = ((hash_cluster_id)%255)/(double)255;
    getRainbowColor(c,r,g,b);
    marker.color.a = 0.25; // Don't forget to set the alpha!
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    
    publisher.publish( marker );
  }
  last_marker_count = clusterCenters.size();

}

// calculate an averaged radius from the previous scan and the current scan
static double calcAveragedRadius(const double prevRadius, const Point3d center, const std::vector<velodyne_pointcloud::PointXYZIR>& obstacles, const std::vector<int> clusterIndices) {
  double alpha = 0.1;
  double detectedRadius = -1;
   for(int i =0; i< clusterIndices.size(); i++) {
	double currentRadius = distance2Points(center, obstacles[i]); 
	if(currentRadius > detectedRadius) {
	  detectedRadius = currentRadius;
	}
      }
      double newRadius = alpha*detectedRadius+(1-alpha)*prevRadius;
  }
  



/*static double distance_velodyne(const velodyne_pointcloud::PointXYZIR& point1, const velodyne_pointcloud::PointXYZIR& point2) {
    return  pcl::euclideanDistance<velodyne_pointcloud::PointXYZIR>(point1, point2);
}*/
#endif //CLUSTER.H