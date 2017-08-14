#include<iostream>
#include<algorithm>
#include<cstdlib>
#include<fstream>
#include <math.h>
#include <vector>
#include <string>
#include <Eigen/Dense>
using namespace std;
using namespace Eigen;
void featurenormalize(MatrixXd &X)
{
    //计算每一维度均值
    MatrixXd meanval = X.colwise().mean();
    RowVectorXd meanvecRow = meanval;
    //样本均值化为0
    X.rowwise() -= meanvecRow;
}
void computeCov(MatrixXd &X, MatrixXd &C)
{
    //计算协方差矩阵C = XTX / n-1;
    C = X.adjoint() * X;
    C = C.array() / X.rows() - 1;

    std::cout<<"The covmatrix is :"<<std::endl;
    std::cout<<C<<std::endl;
}
void computeEig(MatrixXd &C, MatrixXd &vec, MatrixXd &val)
{
    //计算特征值和特征向量，使用selfadjont按照对阵矩阵的算法去计算，可以让产生的vec和val按照有序排列
    SelfAdjointEigenSolver<MatrixXd> eig(C);

    vec = eig.eigenvectors();
    val = eig.eigenvalues();

    std::cout<<"The eigenvalue is :"<< std::endl;
    std::cout<<val<<std::endl;
}
int computeDim(MatrixXd &val)
{
    int dim;
    double sum = 0;
    for (int i = val.rows()-1; i >= 0; --i)
    {
        sum += val(i, 0);
        dim = i;

        if (sum / val.sum() >= 0.95)
            break;
    }
    return val.rows() - dim;
}
int main()
{

    const int m = 6, n = 3;
    MatrixXd X(6, 3), C(3, 3);
    MatrixXd vec, val;

    X(0,0)=105.3;   X(0,1)=114.28;   X(0,2)=-4.59;
    X(1,0)=105.26;  X(1,1)= 114.28;  X(1,2)=-4.36;
    X(2,0)=105.11;  X(2,1)=114.14;   X(2,2)=-4.61;
    X(3,0)=105.5;   X(3,1)=114.43 ;  X(3,2)=-4.57;
    X(4,0)=105.42;  X(4,1)=114.43;   X(4,2)=-4.38;
    X(5,0)=105.52;  X(5,1)=114.43;   X(5,2)= -4.8;


   std::cout<<"The value of the X is"<<std::endl;
   std::cout<<std::endl;
   std::cout<<X<<std::endl;
   std::cout<<std::endl;



    //输入为Eigen::MatrixXf input 输出为covMat
//求取列向量均值
    Eigen::MatrixXf meanVec = X.colwise().mean();

//求取上述的零均值列向量矩阵
    Eigen::MatrixXf zeroMeanMat = X;
//将列向量均值从MatrixXf 转换为行向量 RowVectorXf
    Eigen::RowVectorXf meanVecRow(Eigen::RowVecXf::Map(meanVec.data(),3));
    zeroMeanMat.rowwise() -= meanVecRow;

//计算协方差
    Eigen::MatrixXf covMat = (zeroMeanMat.adjoint()*zeroMeanMat)/double(X.rows()-1);

    std::cout<<"The value of the cov MatrixXf is"<<std::endl;
    std::cout<<std::endl;
    std::cout<<covMat<<std::endl;
    std::cout<<std::endl;



    //pca
//
//    //零均值化
//    featurenormalize(X);
//    //计算协方差
//    computeCov(X, C);
//    //计算特征值和特征向量
//    computeEig(C, vec, val);


//
//    //计算损失率，确定降低维数
//    int dim = computeDim(val);
//    //计算结果
//    MatrixXd res = X * vec.rightCols(dim);
//    //输出结果
//    fout << "the result is " << res.rows() << "x" << res.cols() << " after pca algorithm." << endl;
//    fout << res;
//    system("pause");
    return 0;
}