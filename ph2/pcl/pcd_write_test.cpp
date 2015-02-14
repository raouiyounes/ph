#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/normal_3d.h>


//using namespace pcl;

int main(){

pcl::search::KdTree<pcl::PointXYZ>::Ptr norm_tree(new pcl::search::KdTree<pcl::PointXYZ>());	
pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne; 
return 0;
}
