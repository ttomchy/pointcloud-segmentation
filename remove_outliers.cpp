#include <iostream>
#include<pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

int main(int argc,char**argv){

    if(argc!=2){
        std::cerr<<"please specify command line arg '-r', or '-c' "<<std::endl;
        exit(0);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);


 /*

    //fill in the cloud data;
    cloud->width =5;
    cloud->height=1;
    cloud->points.resize(cloud->width*cloud->height);

    for (size_t i=0; i<cloud->points.size();++i){
        cloud->points[i].x=1024*rand()/(RAND_MAX+1.0f);
        cloud->points[i].y=1024*rand()/(RAND_MAX+1.0f);
        cloud->points[i].z=1024*rand()/(RAND_MAX+1.0f);
    }

*/
    pcl::PCDReader reader;
    //replace the data with our own data;
    reader.read("table_scene_lms400.pcd", *cloud);
    std::cerr << "Point before filtering :" << cloud->width * cloud->height
              << "data points :(" << pcl::getFieldsList(*cloud)
              << ")" << std::endl;

    if(strcmp(argv[1],"-r")==0){
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> outstream;
        //build the filter
        outstream.setInputCloud(cloud);
        outstream.setRadiusSearch(0.8);
        outstream.setMinNeighborsInRadius(2);
        //apply the filter
        outstream.filter(*cloud_filtered);
    }

    else if (strcmp(argv[1],"-c")==0){
        //build the condition
        pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>());
        range_cond->addComparison(
                pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
                    pcl::FieldComparison<pcl::PointXYZ>("z",pcl::ComparisonOps::GT,0.0)));
        range_cond->addComparison(
                pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
                pcl::FieldComparison<pcl::PointXYZ>("z",pcl::ComparisonOps::GT,0.8)
                )
        );

        //build the filter
        pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
        condrem.setCondition(range_cond);
        condrem.setInputCloud(cloud);
        condrem.setKeepOrganized(true);
       // apply the filter
        condrem.filter(*cloud_filtered);
    }

   else{
        std::cerr<<"please specify command line arg '-r' or '-c'"<<std::endl;
        exit(0);
    }


   std::cerr<<"cloud before filtering"<<std::endl;
   for(size_t i=0;i<cloud->points.size();++i){
       std::cerr<<"  "<<cloud->points[i].x
                <<"  "<<cloud->points[i].y
                <<"  "<<cloud->points[i].z
                <<std::endl;
   }

   std::cerr<<"cloud after filtering"<<std::endl;
   for(size_t i=0; i<cloud_filtered->points.size();++i){
       std::cerr<<"  "<<cloud_filtered->points[i].x
                <<"  "<<cloud_filtered->points[i].y
                <<"  "<<cloud_filtered->points[i].z
                <<std::endl;
   }
   return 0;

}
