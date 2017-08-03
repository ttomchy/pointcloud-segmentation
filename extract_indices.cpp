#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>




typedef pcl::PointXYZ PointT;



boost::shared_ptr<pcl::visualization::PCLVisualizer>
simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
boost::shared_ptr<pcl::visualization::PCLVisualizer> customColourVis
        (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);

int
main (int argc, char** argv)
{
    // All the objects needed
    pcl::PCDReader reader;
    pcl::PassThrough<PointT> pass;
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::PCDWriter writer;

    // Datasets
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_p (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr non_cloud_plane (new pcl::PointCloud<PointT>);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);

    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

    // Read in the cloud data
    reader.read ("map3d_split.pcd", *cloud);
    std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;



    //create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(25);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_filtered);
    std::cout<<"cloud after filtering:"<<std::endl;
    std::cerr<<*cloud_filtered<<std::endl;

    //  writer.write<pcl::PointXYZ>("split_map3d_inliers.pcd",*cloud_filtered,false);
    std::cout<<"the size of the in-lier points is:"<<cloud_filtered->points.size ()<<std::endl;

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.1);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;


    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0) {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }


    // Extract the inliers
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
//    extract.setNegative (false);
//    extract.filter (*cloud_p);
//    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
//    writer.write<pcl::PointXYZ> ("table_scene_lms400_plane_1.pcd", *cloud_p, false);
//

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*non_cloud_plane);
//    std::cerr << "PointCloud representing the non_cloud_plane planar component: " << non_cloud_plane->points.size () << " data points." << std::endl;
//    writer.write ("no-table_scene_lms400_plane_1.pcd", *non_cloud_plane, false);


    std::cerr << "PointCloud representing the non_cloud_plane planar component: "
              << non_cloud_plane->points.size () << " data points." << std::endl;
    *cloud_filtered=*non_cloud_plane;
    std::cerr << "PointCloud representing the non_cloud_plane planar component: "
              << cloud_filtered->points.size () << " data points." << std::endl;


//reader.read ("table_scene_lms400.pcd", *cloud);




    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.2); // 10cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (500000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
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







   std::cout<<"the ending"<<std::endl;








//
//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
//
//   // viewer = simpleVis(cloud_p);
//    viewer = customColourVis(cloud_p);
//    while (!viewer->wasStopped ())
//    {
//        viewer->spinOnce (100);
//        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//    }
//


    return (0);
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




/******************可视化点云自定义颜色特征**********************************************************/
/****************************************************************************************************
演示怎样给点云着上单独的一种颜色，可以利用该技术给指定的点云着色，以区别其他的点云，
*****************************************************************************************************/
//点云类型为XYZ类型，customColourVis函数将点云赋值为绿色，
boost::shared_ptr<pcl::visualization::PCLVisualizer> customColourVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    //创建一个自定义的颜色处理器PointCloudColorHandlerCustom对象，并设置颜色为纯绿色
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);
    //addPointCloud<>()完成对颜色处理器对象的传递
    viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    return (viewer);
}
