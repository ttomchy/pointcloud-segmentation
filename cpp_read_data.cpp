#include<iostream>      /////////读取dian.txt文件中的数据 并保存为二维数组
#include<fstream>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/ml/ml.hpp"

#include <iostream>
using namespace cv;
using namespace std;

int main()
{
    int row=14 ,col=5;     //n行 m列

    int i,j;
    float training_data[row][col-1];
    float lables[row];
    ifstream fin("out.txt");         //打开文件//读入数字
    for(i=0;i<row;i++){
        for(j=0;j<col;j++) {
           if(j<col-1){
            fin >> training_data[i][j];
           }else{
               fin>>lables[i];
           }
        }
    }
    fin.close();
    for(i=0;i<row;i++,cout<<endl)                //输出刚刚读入的数据
        for(j=0;j<col-1;j++)
            cout<<std::fixed<<training_data[i][j]<<" ";

    for(i=0;i<row;++i)
        cout<<lables[i]<<endl;

    CvMat trainingDataCvMat = cvMat( row, col-1, CV_32FC1, training_data );


    CvMat responsesCvMat = cvMat( row, 1, CV_32FC1, lables );

    CvRTParams params= CvRTParams(10, 2, 0, false,16, 0, true, 0, 100, 0, CV_TERMCRIT_ITER );

    CvERTrees etrees;
    etrees.train(&trainingDataCvMat, CV_ROW_SAMPLE, &responsesCvMat,
                 Mat(), Mat(), Mat(), Mat(),params);

    double sampleData[4]={0.766716,0.128947,0.104337 ,0.895663};
    Mat sampleMat(4, 1, CV_32FC1, sampleData);
    float r = etrees.predict(sampleMat);
    cout<<endl<<"result:  "<<r<<endl;

    return 0;
}