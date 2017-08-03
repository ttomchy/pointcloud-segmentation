#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>

int main(){

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    //filter in the cloud data;
    pcl::PCDReader reader;
    reader.read<pcl::PointXYZ>("map3d_split.pcd",*cloud);
    std::cerr<<"cloud before filtering"<<std::endl;
    std::cerr<<*cloud<<std::endl;
    //create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(6.0);
    sor.filter(*cloud_filtered);
    std::cout<<"cloud after filtering:"<<std::endl;
    std::cerr<<*cloud_filtered<<std::endl;
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ>("split_map3d_inliers.pcd",*cloud_filtered,false);
    std::cout<<"the size of the in-lier points is:"<<cloud_filtered->points.size ()<<std::endl;
    sor.setNegative(true);
    sor.filter(*cloud_filtered);
    writer.write<pcl::PointXYZ>("split_map3d_outliers.pcd",*cloud_filtered,false);
    std::cout<<"the size of the out-lier points is:"<<cloud_filtered->points.size ()<<std::endl;
    return  0;





}