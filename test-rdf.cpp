#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/ml/ml.hpp"

#include <iostream>
using namespace cv;
using namespace std;

int main( int argc, char** argv )
{






    double trainingData[14][4]={
          {197799, 797150, 0.005051, 0.994949 },
            {588544, 309165,0.102291, 0.897709 },

            {0.584514, 0.408274 ,0.007212 ,0.992788},
            { 0.197799, 0.797150, 0.005051 ,0.994949 },

            {0.791392, 0.208608 ,0.000000, 1.000000 },
            { 0.587254, 0.312074, 0.100672 ,0.899328 },

            {0.521097,0.478903, 0.000000 ,1.000000 },
            { 0.760895, 0.111448, 0.127657, 0.872343},

            {0.791048, 0.127512, 0.081439, 0.918561},
            {0.682655, 0.221412 ,0.095933, 0.904067 },

            {0.689500, 0.210978, 0.099522, 0.900478 },
            {0.418176 ,0.580695 ,0.001129 ,0.998871 } ,

            { 0.692253, 0.232505, 0.075242, 0.924758 },
            {0.766716,0.128947,0.104337 ,0.895663},


};

    CvMat trainingDataCvMat = cvMat( 14, 4, CV_32FC1, trainingData );

    float responses[14] = { 399900, 369000, 539900, 314900, 212000, 239999, 329999,
                            259900, 299900, 499998, 252900, 242900, 573900, 464500,
                           };
    CvMat responsesCvMat = cvMat( 14, 1, CV_32FC1, responses );

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