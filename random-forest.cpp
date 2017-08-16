#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/ml/ml.hpp"
#include <fstream>
#include <iostream>
#include <string>
using namespace cv;
using namespace std;


int CountLines(char *filename)
{
    ifstream ReadFile;
    int n=0;
    string tmp;
    ReadFile.open(filename,ios::in);//ios::in 表示以只读的方式读取文件
    if(ReadFile.fail())//文件打开失败:返回0
    {
        return 0;
    }
    else//文件存在
    {
        while(getline(ReadFile,tmp,'\n'))
        {
            n++;
        }
        ReadFile.close();
        return n;
    }
}


int main( int argc, char** argv )
{

    char filename[512]="feature.txt";
    int LINES=CountLines(filename);
    int NSAMPLES_ALL=LINES;
    cout<<"The number of lines is :"<<LINES<<endl;

    int row=LINES ,col=5;     //n行 m列

    int i,j;
    float training_data[row][col-1];
    float lables[row];
    ifstream fin("feature.txt");         //打开文件//读入数字
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

    CvMat trainingDataCvMat = cvMat( row, col-1, CV_32FC1, training_data );

    CvMat responsesCvMat = cvMat( row, 1, CV_32FC1, lables );

    CvRTParams params= CvRTParams(10, 300, 0, false,5, 0, true, 0, 100, 0, CV_TERMCRIT_ITER );

    CvERTrees etrees;
    etrees.train(&trainingDataCvMat, CV_ROW_SAMPLE, &responsesCvMat,
                 Mat(), Mat(), Mat(), Mat(),params);

    /*******************************testing********************************************/
//
//    double sampleData[4]={0.197799, 0.797150, 0.005051 ,0.994949 };
//    Mat sampleMat(4, 1, CV_32FC1, sampleData);
//    float r = etrees.predict(sampleMat);
//    cout<<endl<<"result:  "<<r<<endl;


 //   CvMat testingdataset = cvMat( row, col-1, CV_32FC1, training_data );


    char filename_test[512]="test_feature.txt";
    int LINES_test=CountLines(filename_test);

    cerr<<"The number of testing data lines is :"<<LINES_test<<endl;

    row=LINES_test ,col=5;     //n行 m列


    float testing_data[row][col-1];
    float testing_lables[row];
    ifstream fin_test("test_feature.txt");         //打开文件//读入数字
    for(int i=0;i<row;i++){
        for(int j=0;j<col;j++) {
            if(j<col-1){
                fin_test >> testing_data[i][j];
            }else{
                fin_test>>testing_lables[i];
            }
        }
    }
    fin.close();

    Mat testing_dataCvMat= Mat( row, col-1, CV_32FC1, testing_data );
    Mat testing_lablesCvMat = Mat( row, 1, CV_32FC1, testing_lables );

    std::cerr<<"The size of the testing_dataCvMat is :"<<testing_dataCvMat.rows
             <<" "<<testing_dataCvMat.cols<<std::endl;
    std::cerr<<"The size of the testing_lablesCvMat is:"<< testing_lablesCvMat.rows
             <<" "<<testing_lablesCvMat.cols <<std::endl;

    // 预测误差

    double test_hr = 0;

    for (int i=0; i<LINES_test; i++)
    {
        double r;
        Mat sample = testing_dataCvMat.rowRange(i,i+1).clone();
      //  std::cout<<"The value  of the testing_dataCvMat("<<i<<") is:"<<sample<<std::endl;

        
        r = etrees.predict(sample);
        std::cout<<"The value of r is :"<<i<<" "<<r<<" "<<testing_lablesCvMat.at<float>(i,0) <<std::endl;
        r = fabs((double)r - testing_lablesCvMat.at<float>(i,0)) <= FLT_EPSILON ? 1 : 0;

        test_hr += r;
    }

    test_hr /=LINES_test;

    cerr<<"The accuracy rate is :"<<test_hr<<endl;


    return 0;
}