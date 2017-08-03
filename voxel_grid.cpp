#include <iostream>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

int main() {

    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

    //fill in the cloud data
    pcl::PCDReader reader;
    //replace the data with our own data;
    reader.read("table_scene_lms400.pcd", *cloud);
    std::cerr << "Point before filtering :" << cloud->width * cloud->height
              << "data points :(" << pcl::getFieldsList(*cloud)
              << ")" << std::endl;
    //create the filtering object
    pcl::VoxelGrid<pcl::PCLPointCloud2>sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.01f,0.01f,0.01f);
    sor.filter(*cloud_filtered);
    std::cerr<<"Points cloud after filtering:"<<cloud_filtered->width*cloud_filtered->height
             <<"data points ("<<pcl::getFieldsList(*cloud_filtered)<<")"
             <<std::endl;
    pcl::PCDWriter writer;
    writer.write("voxel-downsampled.pcd",*cloud_filtered,Eigen::Vector4f::Zero(),
                 Eigen::Quaternionf::Identity (), false);
    return 0;
}
