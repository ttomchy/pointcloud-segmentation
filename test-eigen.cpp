
#include <math.h>
#include <iostream>
#include <vector>
#include <string>

#include <Eigen/Dense>
using namespace Eigen;
int main()
{
    // reference: https://stackoverflow.com/questions/15138634/eigen-is-there-an-inbuilt-way-to-calculate-sample-covariance
    std::vector<std::vector<float>> vec{
            { 105.3, 114.28 , -4.59},
            { 105.26 ,114.28 , -4.36},
            { 105.11, 114.14 , -4.61},
            { 105.5 ,114.43 , -4.57},
            { 105.42, 114.43,  -4.38},
            { 105.52, 114.43 ,  -4.8}
    };

    const int rows{ 6}, cols{ 3 };

    std::vector<float> vec_;
    for (int i = 0; i < rows; ++i) {
        vec_.insert(vec_.begin() + i * cols, vec[i].begin(), vec[i].end());
    }
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> m(vec_.data(), rows, cols);

    fprintf(stderr, "source matrix:\n");
    std::cout << m << std::endl;

    fprintf(stdout, "\nEigen implement:\n");
    const int nsamples = rows;
    float scale = 1. / (nsamples - 1);

    Eigen::MatrixXf mean = m.colwise().mean();
    std::cout << "print mean: " << std::endl << mean << std::endl;

    Eigen::MatrixXf tmp(rows, cols);
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            tmp(y, x) = m(y, x) - mean(0, x);
        }
    }
    //std::cout << "tmp: " << std::endl << tmp << std::endl;

    Eigen::MatrixXf covar = (tmp.adjoint() * tmp) / float(nsamples - 1);
    std::cout << "print covariance matrix: " << std::endl << covar << std::endl;
    EigenSolver<Matrix3f> es(covar);
    Matrix3f D = es.pseudoEigenvalueMatrix();

    std::cout<<"The eigenvalue is :"<< std::endl;
    std::cout<<D<<std::endl;


    return 0;
}


















