#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>


boost::shared_ptr<pcl::visualization::PCLVisualizer>
simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
int main() {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    //filter in the cloud data;
    pcl::PCDReader reader;
    reader.read<pcl::PointXYZ>("map3d_split.pcd", *cloud);
    std::cerr << "cloud before removing the ground" << std::endl;
    std::cerr<<"The size of the cloud"<<cloud->points.size()<<std::endl;

    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits  (-0.4,0);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_filtered);

    std::cerr<<" cloud after remove the ground "<<std::endl;
    std::cerr<<"The size of the cloud_filtered"<<cloud_filtered->points.size()<<std::endl;

    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ>("no-ground.pcd",*cloud_filtered,false);



    for(size_t i=0; i<cloud->points.size();++i){
        std::cout<<cloud->points[i].x
                 <<" "<<cloud->points[i].y
                 <<" "<<cloud->points[i].z
                 <<std::endl;
    }

    std::cerr<<"Succssful write the no ground data into no-ground.pcd"<<std::endl;



    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

    viewer = simpleVis(cloud_filtered);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }


    return 0;

}


boost::shared_ptr<pcl::visualization::PCLVisualizer>
simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    //viewer->addCoordinateSystem (1.0, "global");
    viewer->initCameraParameters ();
    return (viewer);
}