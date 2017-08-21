#include <iostream>
#include <iomanip>

#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>

#include "./include/features.h"

using namespace Eigen;
typedef pcl::PointXYZI PclPoint;


template<typename T>
bool swap_if_gt(T& a, T& b) {
    if (a < b) {
        std::swap(a, b);
        return true;
    }
    return false;
}

using namespace std;


void EigenvalueBasedDescriptor( pcl::PointCloud<PclPoint> & segment,float local_density, float lable){


    vector<double> feature_vec;

    PclPoint minPt, maxPt;

    //获取坐标极值
    pcl::getMinMax3D(segment, minPt, maxPt);
    feature_vec.push_back(maxPt.z-minPt.z);//delta z

    feature_vec.push_back(local_density);//local point density


    int kNPoints=segment.points.size();//get the number of the points

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

    const int nsamples = rows;

    Eigen::MatrixXf mean = m.colwise().mean();

    Eigen::MatrixXf tmp(rows, cols);
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            tmp(y, x) = m(y, x) - mean(0, x);
        }
    }

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
    feature_vec.push_back(sum_eigenvalues); //sum of eigenvalues.

    e1 = e1 / sum_eigenvalues;
    e2 = e2 / sum_eigenvalues;
    e3 = e3 / sum_eigenvalues;

    // std::cout<<"The value of the gigenvalue e1 is :"<<e1<<std::endl;
    //std::cout<<"The value of the gigenvalue e2 is :"<<e2<<std::endl;
    // std::cout<<"The value of the gigenvalue e3 is :"<<e3<<std::endl;

    if ((e1 == e2) || (e2 == e3) || (e1 == e3)) {
        std::cerr << "The eigenvalue should not be equal!!!" << std::endl;
    }

    const double sum_of_eigenvalues = e1 + e2 + e3;
    double kOneThird = 1.0 / 3.0;

    if (sum_of_eigenvalues == 0.0) {
        std::cerr << "The sum of the eigenvalue is 0.0" << std::endl;
    }

    feature_vec.push_back((e1 - e2) / e1);//linearity
    feature_vec.push_back((e2 - e3) / e1);//planarity
    feature_vec.push_back(e3 / e1);//scattering
    //feature_vec.push_back(std::pow(e1 * e2 * e3, kOneThird));//omnivariance
    feature_vec.push_back((e1 - e3) / e1);//anisotropy
    //feature_vec.push_back((e1 * std::log(e1)) + (e2 * std::log(e2)) + (e3 * std::log(e3)));//eigen_entropy
    //feature_vec.push_back(e3 / sum_of_eigenvalues);//change_of_curvature
    feature_vec.push_back( int (lable));

    Feature eigenvalue_feature;

    vector<double>::iterator t;
    for (t = feature_vec.begin(); t != feature_vec.end(); t++) {

      std::cout <<std::fixed<< *t << " ";

    }
    std::cout<<std::endl;




}


int main (int argc, char** argv) {

    pcl::PointCloud<PclPoint>::Ptr origin_cloud (new pcl::PointCloud<PclPoint>);

    //Read the point cloud.
    pcl::PCDReader reader;
   //reader.read ("ori-dataset.pcd", *origin_cloud); // Remember to download the file first!
    reader.read ("testdataset.pcd", *origin_cloud);
    std::cerr << "PointCloud before filtering: " << origin_cloud->width * origin_cloud->height
              << " data points (" << pcl::getFieldsList (*origin_cloud) << ")."<<std::endl;


    //创建KdTreeFLANN对象，并把创建的点云设置为输入,创建一个searchPoint变量作为查询点
    pcl::KdTreeFLANN<PclPoint> kdtree;
    //设置搜索空间
    kdtree.setInputCloud (origin_cloud);
    //设置查询点并赋随机值

    //For every point we try to find its neighbor points.
    for (size_t i = 0; i < origin_cloud->points.size (); ++i) {
        PclPoint searchPoint;
        searchPoint.x = origin_cloud->points[i].x;
        searchPoint.y = origin_cloud->points[i].y;
        searchPoint.z = origin_cloud->points[i].z;
        searchPoint.intensity=origin_cloud->points[i].intensity;

        //This is the test of one point,try to find its neighbors
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

        float radius = 0.6;//Here we set the radious is 0.4
        float local_density=0;
        pcl::PointCloud<PclPoint> segment;

        if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) >
            0){

            PclPoint searchPoint_res;
            pcl::PointXYZ point_local;

            for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {

//
//                std::cout << " "  << origin_cloud->points[ pointIdxRadiusSearch[i] ].x
//                          << " " << origin_cloud->points[ pointIdxRadiusSearch[i] ].y
//                          << " " << origin_cloud->points[ pointIdxRadiusSearch[i] ].z
//                          << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
                local_density += pointRadiusSquaredDistance[i];

                searchPoint_res.x = origin_cloud->points[pointIdxRadiusSearch[i]].x;
                searchPoint_res.y = origin_cloud->points[pointIdxRadiusSearch[i]].y;
                searchPoint_res.z = origin_cloud->points[pointIdxRadiusSearch[i]].z;
                searchPoint_res.intensity = origin_cloud->points[pointIdxRadiusSearch[i]].intensity;
                segment.push_back(searchPoint_res);

                point_local.x= origin_cloud->points[pointIdxRadiusSearch[i]].x;
                point_local.y= origin_cloud->points[pointIdxRadiusSearch[i]].y;
                point_local.z= origin_cloud->points[pointIdxRadiusSearch[i]].z;

            }
            local_density=local_density/ pointIdxRadiusSearch.size();

        }

        int num_points=segment.points.size();//get the number of the points
        // if the point's neighborhoood points is two low ,we consider it must be a noise point
        if (num_points<=2){
            ;
        }
        else {

            EigenvalueBasedDescriptor(segment,local_density,searchPoint.intensity);
        }
    }

    std::cerr<<"The ending !!!"<<std::endl;

    return 0;
}
