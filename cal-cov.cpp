#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include "./include/features.h"
#include <pcl/kdtree/kdtree_flann.h> //kdtree类定义头文件
#include <Eigen/Dense>
using namespace Eigen;
typedef pcl::PointXYZI PclPoint;
typedef pcl::PointCloud<PclPoint> PointCloud;

#include <fstream>


PclPoint centroid;

template<typename T>
bool swap_if_gt(T& a, T& b) {
    if (a < b) {
        std::swap(a, b);
        return true;
    }
    return false;
}
int too_less_points=0;
using namespace std;
void EigenvalueBasedDescriptor( pcl::PointCloud<PclPoint> & segment){

   //  std::cout << "PointCloud before processing: " << segment.width * segment.height
   //           << " data points (" << pcl::getFieldsList (segment) << ")."<<std::endl;

    int kNPoints=segment.points.size();//get the number of the points
    int n=3;

        std::vector<std::vector<float>> vec;
        std::vector<float> v1;

        for (int i = 0; i < kNPoints; ++i) {
            v1.push_back(segment.points[i].x);
            v1.push_back(segment.points[i].y);
            v1.push_back(segment.points[i].z);
            vec.push_back(v1);
            v1.clear();
        }

        const int rows{kNPoints}, cols{3};

        std::vector<float> vec_;
        for (int i = 0; i < rows; ++i) {
            vec_.insert(vec_.begin() + i * cols, vec[i].begin(), vec[i].end());
        }
        Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> m(vec_.data(), rows, cols);

        //fprintf(stderr, "source matrix:\n");
        //std::cout << m << std::endl;

        // fprintf(stdout, "\nEigen implement:\n");
        const int nsamples = rows;
        float scale = 1. / (nsamples - 1);

        Eigen::MatrixXf mean = m.colwise().mean();
        //  std::cout << "print mean: " << std::endl << mean << std::endl;

        Eigen::MatrixXf tmp(rows, cols);
        for (int y = 0; y < rows; ++y) {
            for (int x = 0; x < cols; ++x) {
                tmp(y, x) = m(y, x) - mean(0, x);
            }
        }
        //std::cout << "tmp: " << std::endl << tmp << std::endl;

        Eigen::MatrixXf covar = (tmp.adjoint() * tmp) / float(nsamples - 1);
        //std::cout << "print covariance matrix: " << std::endl << covar << std::endl;
        EigenSolver<Matrix3f> es(covar);
        Matrix3f D = es.pseudoEigenvalueMatrix();

        //std::cout<<"The eigenvalue is :"<< std::endl;
        //std::cout<<D<<std::endl;

        double e1 = D(0, 0);
        double e2 = D(1, 1);
        double e3 = D(2, 2);

        // Sort eigenvalues from smallest to largest.
        swap_if_gt(e1, e2);
        swap_if_gt(e1, e3);
        swap_if_gt(e2, e3);
        //std::cout<<"The eigenvalue is e1:"<<  e1<< std::endl;
        //std::cout<<"The eigenvalue is e2:"<<  e2<< std::endl;
        //std::cout<<"The eigenvalue is e3:"<<  e3<< std::endl;
        // Normalize eigenvalues.
        double sum_eigenvalues = e1 + e2 + e3;
        e1 = e1 / sum_eigenvalues;
        e2 = e2 / sum_eigenvalues;
        e3 = e3 / sum_eigenvalues;

        // std::cout<<"The value of the gigenvalue e1 is :"<<e1<<std::endl;
        //std::cout<<"The value of the gigenvalue e2 is :"<<e2<<std::endl;
        // std::cout<<"The value of the gigenvalue e3 is :"<<e3<<std::endl;

        if ((e1 == e2) || (e2 == e3) || (e1 == e3)) {
            std::cerr << "The eigenvalue should not be equal!!!" << std::endl;
        }

        // Store inside features.
        const double sum_of_eigenvalues = e1 + e2 + e3;
        constexpr double kOneThird = 1.0 / 3.0;

        if (e1 == 0.0) {
            std::cerr << "The sum of the eigenvalue is 0.0" << std::endl;
        }
        if (sum_of_eigenvalues == 0.0) {
            std::cerr << "The sum of the eigenvalue is 0.0" << std::endl;
        }

        vector<double> feature_vec;

        feature_vec.push_back((e1 - e2) / e1);//linearity
        feature_vec.push_back((e2 - e3) / e1);//planarity
        feature_vec.push_back(e3 / e1);//scattering
        //feature_vec.push_back(std::pow(e1 * e2 * e3, kOneThird));//omnivariance
        feature_vec.push_back((e1 - e3) / e1);//anisotropy
//        feature_vec.push_back((e1 * std::log(e1)) + (e2 * std::log(e2)) + (e3 * std::log(e3)));//eigen_entropy
//        feature_vec.push_back(e3 / sum_of_eigenvalues);//change_of_curvature

        Feature eigenvalue_feature;

        PclPoint point_min, point_max;

        double diff_x, diff_y, diff_z;

        diff_x = point_max.x - point_min.x;
        diff_y = point_max.y - point_min.y;
        diff_z = point_max.z - point_min.z;

        if (diff_z < diff_x && diff_z < diff_y) {
            feature_vec.push_back(0.2);//pointing_up

        } else {
            feature_vec.push_back(0.0);//pointing_up
        }

        vector<double>::iterator t;
        for (t = feature_vec.begin(); t != feature_vec.end(); t++) {

            std::cout << *t << " ";//style="white-space:pre">  //不用科学计数法

        }
        //std::cout<<segment.points[].<<std::endl;


}

int main (int argc, char** argv) {

    pcl::PointCloud<PclPoint>::Ptr origin_cloud (new pcl::PointCloud<PclPoint>);


    // Fill in the cloud data
    pcl::PCDReader reader;
    // Replace the path below with the path where you saved your file
    reader.read ("ori-dataset.pcd", *origin_cloud); // Remember to download the file first!

    std::cerr << "PointCloud before filtering: " << origin_cloud->width * origin_cloud->height
              << " data points (" << pcl::getFieldsList (*origin_cloud) << ")."<<std::endl;


    //创建KdTreeFLANN对象，并把创建的点云设置为输入,创建一个searchPoint变量作为查询点
    pcl::KdTreeFLANN<PclPoint> kdtree;
    //设置搜索空间
    kdtree.setInputCloud (origin_cloud);
    //设置查询点并赋随机值
    //122.823 122.943 -8.53333

    for (size_t i = 0; i < origin_cloud->points.size (); ++i) {
        PclPoint searchPoint;
        searchPoint.x = origin_cloud->points[i].x;
        searchPoint.y = origin_cloud->points[i].y;
        searchPoint.z = origin_cloud->points[i].z;
        searchPoint.intensity=origin_cloud->points[i].intensity;
//    searchPoint.x =  105.3;
//    searchPoint.y =  114.28;
//    searchPoint.z = -4.59;
    /**********************************************************************************
     下面的代码展示查找到给定的searchPoint的某一半径（随机产生）内所有近邻，重新定义两个向量
     pointIdxRadiusSearch  pointRadiusSquaredDistance来存储关于近邻的信息
     ********************************************************************************/
        // 半径 R内近邻搜索方法

        std::vector<int> pointIdxRadiusSearch;           //存储近邻索引
        std::vector<float> pointRadiusSquaredDistance;   //存储近邻对应距离的平方

        //float radius = 256.0f * rand () / (RAND_MAX + 1.0f);   //随机的生成某一半径
        //打印输出
        float radius = 0.4;
//        std::cout << std::endl;
//        std::cout << "Neighbors within radius search at (" << searchPoint.x
//                  << " " << searchPoint.y
//                  << " " << searchPoint.z
//                  << ") with radius=" << radius << std::endl;

        pcl::PointCloud<PclPoint> segment;
        if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) >
            0)  //执行半径R内近邻搜索方法
        {
            PclPoint searchPoint_res;
            for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
//                std::cout << "    " << origin_cloud->points[pointIdxRadiusSearch[i]].x
//                          << " " << origin_cloud->points[pointIdxRadiusSearch[i]].y
//                          << " " << origin_cloud->points[pointIdxRadiusSearch[i]].z
//                          << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
                searchPoint_res.x = origin_cloud->points[pointIdxRadiusSearch[i]].x;
                searchPoint_res.y = origin_cloud->points[pointIdxRadiusSearch[i]].y;
                searchPoint_res.z = origin_cloud->points[pointIdxRadiusSearch[i]].z;
                searchPoint_res.intensity = origin_cloud->points[pointIdxRadiusSearch[i]].intensity;
                segment.push_back(searchPoint_res);
            }
        }

        int num_points=segment.points.size();//get the number of the points

        if (num_points<=2){
            ;
        }
        else {
            EigenvalueBasedDescriptor(segment);
            std::cout << searchPoint.intensity << std::endl;
        }
    }


    std::cerr<<"Ending !!!"<<std::endl;
    std::cerr<<"The ending !!!"<<std::endl;



    return 0;
}
