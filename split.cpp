
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
int point_size=200000;
int
main (int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_split (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ> cloud_res;

    // Fill in the cloud data
    pcl::PCDReader reader;
    // Replace the path below with the path where you saved your file
    reader.read ("map3d.pcd", *cloud); // Remember to download the file first!

    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
              << " data points (" << pcl::getFieldsList (*cloud) << ")."<<std::endl;


    cloud_split->width  = 500000;
    cloud_split->height = 1;
    cloud_split->points.resize (cloud_split->width * cloud_split->height);

    for (int i = 0; i <cloud->points.size (); ++i)
    {
        double val_x=cloud->points[i].x;
        double val_y=cloud->points[i].y;


        if (( (( val_x<-8)& ( val_x>-16))) &( (( val_y<19.5)& ( val_y>14)))      ){
            pcl::PointXYZ o;
            o.x=cloud->points[i].x;
            o.y=cloud->points[i].y;
            o.z=cloud->points[i].z;
            cloud_res.push_back(o);
      }

    }

    std::cerr << "Cloud after spliting: " << cloud_res.width * cloud_res.height
              << " data points (" << pcl::getFieldsList (cloud_res) << ").";
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ> ("map3d_split.pcd", cloud_res, false);
    std::cout<<std::endl;

    return (0);
}


