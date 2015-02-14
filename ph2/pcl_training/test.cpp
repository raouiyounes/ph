#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/filters/passthrough.h"
 #include <pcl/features/feature.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
int main(int argc, char** argv)
{

pcl::PointCloud<pcl::PointXYZ> cloud, cloud_filtered;
// Fill in the cloud data

cloud.width = 5;
cloud.height = 1;
cloud.points.resize(cloud.width * cloud.height);

for (size_t i = 0; i < cloud.points.size(); ++i)
{
cloud.points[i].x = 1024 * rand() / (RAND_MAX + 1.0);
cloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0);
cloud.points[i].z = 1024 * rand() / (RAND_MAX + 1.0);
}
std::cerr << "Cloud before filtering: " << std::endl;

for (size_t i = 0; i < cloud.points.size(); ++i)

std::cerr << " " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;
pcl::PassThrough<pcl::PointXYZ> pass; // Create the filtering object
pass.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(cloud));
pass.setFilterFieldName ("z");
pass.setFilterLimits (0.0, 1.0);
pass.filter(cloud_filtered);
std::cerr << "Cloud after filtering: " << std::endl;

for (size_t i = 0; i < cloud_filtered.points.size(); ++i)

std::cerr << " " << cloud_filtered.points[i].x << " " << cloud_filtered.points[i].y << " " <<cloud_filtered.points[i].z << std::endl;

return (0);
}


