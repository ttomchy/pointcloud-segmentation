#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/time.h>   // TicToc


boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);



int
main (int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);


    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    //pcl::visualization::CloudViewer viewer ("Viewer");

    // Fill in the cloud data
    pcl::PCDReader reader;
    // Replace the path below with the path where you saved your file
    reader.read ("map3d.pcd", *cloud); // Remember to download the file first!
    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
              << " data points (" << pcl::getFieldsList (*cloud) << ").";
    // total about 279971 data points (x y z)
    // all the points' z values are negative


    /*
    for (size_t i = 0; i < cloud->points.size (); ++i)
        std::cout << "    " << cloud->points[i].x << " "
                  << cloud->points[i].y << " "
                  << cloud->points[i].z << std::endl;

    */

    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-3,-0.45);
   // pass.setFilterLimitsNegative (true);//return the value out of the range. default is negative .
    pass.filter (*cloud_filtered);


    pcl::io::savePCDFileASCII ("test_pcd.pcd", *cloud_filtered);
    std::cerr << "Saved " << cloud_filtered->points.size () << " data points to test_pcd.pcd." << std::endl;
    std::cerr << "Cloud after filtering: " << std::endl;



    uint8_t r(255), g(15), b(15);

    for (size_t i = 0; i < cloud->points.size (); ++i){

        pcl::PointXYZRGB point;
        point.x=cloud->points[i].x;
        point.y=cloud->points[i].y;
        point.z=cloud->points[i].z;

        uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                        static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
        point.rgb = *reinterpret_cast<float*>(&rgb);
        point_cloud_ptr->points.push_back (point);

        if (point.z < 0.0)
        {
            r -= 24;
            g += 24;
        }
        else
        {
            g -= 12;
            b += 12;
        }

    }

    point_cloud_ptr->width = (int) point_cloud_ptr->points.size ();
    point_cloud_ptr->height = 1;






    viewer = rgbVis(point_cloud_ptr);


    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }




    return (0);
}


/*****************************可视化点云颜色特征******************************************************/
/**************************************************************************************************
多数情况下点云显示不采用简单的XYZ类型，常用的点云类型是XYZRGB点，包含颜色数据，除此之外，还可以给指定的点云定制颜色
 以示得点云在视窗中比较容易区分。点赋予不同的颜色表征其对应的Z轴值不同，PCL Visualizer可根据所存储的颜色数据为点云
 赋色， 比如许多设备kinect可以获取带有RGB数据的点云，PCL Vizualizer可视化类可使用这种颜色数据为点云着色，rgbVis函数中的代码
用于完成这种操作。
 ***************************************************************************************************/
/**************************************************************************
 与前面的示例相比点云的类型发生了变化，这里使用的点云带有RGB数据的属性字段，
****************************************************************************/
boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    /***************************************************************************************************************
    设置窗口的背景颜色后，创建一个颜色处理对象，PointCloudColorHandlerRGBField利用这样的对象显示自定义颜色数据，PointCloudColorHandlerRGBField
     对象得到每个点云的RGB颜色字段，
    **************************************************************************************************************/

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    return (viewer);
}