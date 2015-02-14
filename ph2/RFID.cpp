#include <iostream>
#include <math.h>
#include <string.h>
#include <vector.h>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include "RFID.h"

using namespace std;
using namespace Eigen;


int  RFID::RFID_cluster_data_with_pcl(){

pcl::PCDReader reader;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
reader.read ("test_rfid.pcd", *cloud);
 pcl::ExtractIndices<pcl::PointXYZ> extract;
  int i=0, nr_points = (int)cloud_filtered->points.size();

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PCDWriter writer;
std::cout<< "point befor filtering:"<<cloud->points.size()<<"data points"<<endl;
  pcl::VoxelGrid<pcl::PointXYZ> vg;


vg.setInputCloud(cloud);
vg.setLeafSize(0.01f,0.01f,0.01f);
vg.filter(*cloud_filtered);
std::cout<<"point after filtering"<<cloud_filtered->points.size()<<"data points"<<endl;

 pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
 
  std::vector<pcl::PointIndices> cluster_indices;

  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (10);
  ec.setMaxClusterSize (100);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract(cluster_indices);

int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "cloud_cluster_" << j << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    j++;
  }

 

  return 0;

}

/*

    void Mapping(VectorXd U){
      VectorXd Xtag_m(20),Ytag_m(20);
        float r=20.5;
	int ntag,k=0;
       Xr(0)=U(0)*cos(Xr(2))+U(1)*sin(Xr(2));
        Xr(1)=-U(0)*sin(Xr(2))+U(1)*cos(Xr(2));
        for(int i=0;i<ntag;i++)
            if( ((Xr(0)-Xtag(i))+(Xr(1)-Ytag(i)))<r){
            Xtag_m(k)=Xtag(i);
            Ytag_m(k)=Ytag(i);
	    k++;            
}
        Xtag=Xtag_m;
        Ytag=Ytag_m;
    }

    void rotation_direct(VectorXd X,VectorXd U){
X(0)=U(0)*cos(X(3))+U(1)*sin(X(3));
X(1)=-U(0)*sin(X(3))+U(1)*sin(X(3));
X(2)=X(2)+U(2);
}

 int sizeOf(VectorXd tag_estimate){
   int counter=0,i=0;
while(tag_estimate(i++)!=0){
counter++;
}
return counter;
}

VectorXd  robot_position_estimation(VectorXd X){
  VectorXd tag_estimate_coordinate(20), Xestimate(3);
 VectorXf noise(3) ;
int i,j;
//tag_estimate_coordinate=observation_estimation(X);
Eigen::MatrixXd X_i(sizeOf(tag_estimate_coordinate),3);

for(i=0;i<sizeOf(tag_estimate_coordinate);i++){
noise=VectorXf::Random(3);
 X_i(i,0)=X(0)+noise(0);
 X_i(i,1)=X(1)+noise(1);
 X_i(i,2)=X(2)+noise(2);
 }
for(j=0;j<sizeOf(tag_estimate_coordinate);j++){
Xestimate=Xestimate+X_i(j,0)*tag_estimate_coordinate(j);
}
return Xestimate/sizeOf(tag_estimate_coordinate);
}


void  tag_pos_estimation(VectorXd X, VectorXd U, VectorXd tag_coord){
  VectorXd offset(3);
  MatrixXd tag_particle(20,3);
    float r=24.0,distance;
    int i,number_particle=50;
    if( sqrt( pow(X(0)-tag_coord(0),2)+pow(X(1)-tag_coord(1)),2)<pow(r,2)){

      for(i=0;i<number_particle;i++){
	offset=VectorXf::Random(3);
	tag_particle(i,0)=tag_coord(0)+offset(0);
	tag_particle(i,1)=tag_coord(1)+offset(1);

}

      for(i=0;i<number_particle;i++){
	if( distance=sqrt( (X(0)-tag_particle(i,0))*(X(0)-tag_particle(i,0))+(X(1)-tag_particle(i,0))*(X(1)-tag_particle(i,0)))<r*r){
	    W_tag(i)=modelSensor(distance);	
}
}
     void  sensorModel(float x,float y, float theta){
	    int i,Number_polygone_parts=10;
	      while(i<Number_polygone_parts){
	      poly[i++]=x+r*cos(theta);
	      poly[i++]=y+r*sin(theta);
	      theta+=360/Number_polygone_parts;
}





*/






    


