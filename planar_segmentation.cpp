//#include <iostream>
//#include <pcl/ModelCoefficients.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/segmentation/sac_segmentation.h>
//
//
//
//
//int
//main (int argc, char** argv)
//{
//
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);
//
//    //filter in the cloud data;
//    pcl::PCDReader reader;
//    reader.read<pcl::PointXYZ>("split_map3d_inliers.pcd", *cloud);
//    std::cerr << "cloud before removing the ground" << std::endl;
//    std::cout<<"The size of the cloud"<<cloud->points.size()<<std::endl;
//
//
//
//
//
//
////    // Fill in the cloud data
////    cloud->width  = 15;
////    cloud->height = 1;
////    cloud->points.resize (cloud->width * cloud->height);
////
////    // Generate the data
////    for (size_t i = 0; i < cloud->points.size (); ++i)
////    {
////        cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
////        cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
////        cloud->points[i].z = 1.0;
////    }
////
////    // Set a few outliers
////    cloud->points[0].z = 2.0;
////    cloud->points[3].z = -2.0;
////    cloud->points[6].z = 4.0;
////
////    std::cerr << "Point cloud data: " << cloud->points.size () << " points" << std::endl;
////    for (size_t i = 0; i < cloud->points.size (); ++i)
////        std::cerr << "    " << cloud->points[i].x << " "
////                  << cloud->points[i].y << " "
////                  << cloud->points[i].z << std::endl;
////
//
//
//    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
//    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
//    // Create the segmentation object
//    pcl::SACSegmentation<pcl::PointXYZ> seg;
//    // Optional
//    seg.setOptimizeCoefficients (true);
//    // Mandatory
//    seg.setModelType (pcl::SACMODEL_PLANE);
//    seg.setMethodType (pcl::SAC_RANSAC);
//    seg.setDistanceThreshold (0.01);
//
//    seg.setInputCloud (cloud);
//    seg.segment (*inliers, *coefficients);
//
//    if (inliers->indices.size () == 0)
//    {
//        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
//        return (-1);
//    }
//
//    std::cerr << "Model coefficients: " << coefficients->values[0] << " "
//              << coefficients->values[1] << " "
//              << coefficients->values[2] << " "
//              << coefficients->values[3] << std::endl;
//
//    std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
//
//
//    std::cout<<" cloud after remove the ground "<<std::endl;
//
//
//
//    pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);
//    pcl::PCDWriter writer;
//    writer.write<pcl::PointXYZ>("plane-segment-no-ground.pcd",*final,false);
//    std::cout<<"Succssful write the no ground data into plane-segment-no-ground.pcd"<<std::endl;
//
//
//
//
//
////    for (size_t i = 0; i < inliers->indices.size (); ++i)
////        std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
////                  << cloud->points[inliers->indices[i]].y << " "
////                  << cloud->points[inliers->indices[i]].z << std::endl;
//
//    return (0);
//}



#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

boost::shared_ptr<pcl::visualization::PCLVisualizer>
simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");

    std::cout<<"the value of the net is :"<<cloud->points.x <<std::endl;

//    pcl::PCDWriter writer;
//    writer.write<pcl::PointXYZ>("plane-no-ground.pcd",cloud,false);
//    std::cout<<"Succssful write the plane no ground data into no-ground.pcd"<<std::endl;

    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    //viewer->addCoordinateSystem (1.0, "global");
    viewer->initCameraParameters ();
    return (viewer);
}

int
main(int argc, char** argv)
{
    // initialize PointClouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);

    //filter in the cloud data;
    pcl::PCDReader reader;
    reader.read<pcl::PointXYZ>("split_map3d_inliers.pcd", *cloud);
    std::cerr << "cloud before removing the ground" << std::endl;
    std::cout<<"The size of the cloud"<<cloud->points.size()<<std::endl;


    std::vector<int> inliers;

    // created RandomSampleConsensus object and compute the appropriated model
    pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr
            model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZ> (cloud));
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
            model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud));
    if(pcl::console::find_argument (argc, argv, "-f") >= 0)
    {
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
        ransac.setDistanceThreshold (.01);
        ransac.computeModel();
        ransac.getInliers(inliers);
    }
    else if (pcl::console::find_argument (argc, argv, "-sf") >= 0 )
    {
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_s);
        ransac.setDistanceThreshold (.01);
        ransac.computeModel();
        ransac.getInliers(inliers);
    }

//    // Extract the planar inliers from the input cloud
//    extract.setInputCloud (cloud_filtered);
//    extract.setIndices (inliers_plane);
//    extract.setNegative (false);
//
//    // Write the planar inliers to disk
//    pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
//    extract.filter (*cloud_plane);
//    std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
//    writer.write ("table_scene_mug_stereo_textured_plane.pcd", *cloud_plane, false);








    // creates the visualization object and adds either our orignial cloud or all of the inliers
    // depending on the command line arguments specified.

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    if (pcl::console::find_argument (argc, argv, "-f") >= 0 || pcl::console::find_argument (argc, argv, "-sf") >= 0)
        viewer = simpleVis(final);
    else
        viewer = simpleVis(cloud);
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    return 0;
}