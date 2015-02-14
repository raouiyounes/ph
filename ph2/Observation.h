/*
 * Observation.h
 *
 *  Created on: 9 févr. 2015
 *      Author: younes
 */
#include <iostream>
#include <vector.h>
#include <cv.h>
#include <fstream>
#include <highgui.h>
#include <iostream>
#include "Vision.h"
#include <string>
#include <eigen3/Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <flann/flann.h>
#include <flann/io/hdf5.h>
#include <boost/filesystem.hpp>

#ifndef OBSERVATION_H_
#define OBSERVATION_H_

namespace ph2 {

class Observation {
public:
	int number_of_lines;
	int number_of_column;
	int robot_state;
	Eigen::Matrix3f covariance_matrix;
	VectorXd descriptor_i;
	Observation(int nb_l,int nb_c, VectorXd V){
		number_of_lines=nb_l;
		number_of_column=nb_c;
		descriptor_i=V;
	}

	virtual ~Observation();

	void covariance_computation(){
		 pcl::PointCloud<pcl::PointXYZ> cloud;
		 Eigen::Vector4f xyz_centroid;
		 compute3DCentroid (cloud, xyz_centroid);
		computeCovarianceMatrix (cloud, xyz_centroid, covariance_matrix);

	}
};

 /* namespace ph2 */

#endif /* OBSERVATION_H_ */
