//
// Created by 崔瑜翔 on 2018/3/20.
//

#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "GeometryTypes.h"

using namespace cv;
using namespace std;

class Marker
{
public:
    Marker();

    friend bool operator<(const Marker& M1,const Marker& M2);
    friend ostream & operator << (ostream &str,const Marker &M);

    static Mat rotate(Mat in);
    static int hammDistMarker(Mat bits);
    static int mat2id(const cv::Mat &bits);
    static int getMarkerId(Mat &in,int &nRotations);

public:

    int id;

    Transformation transformation;

    vector<Point2f> points;

    void drawContour(Mat& image,Scalar color = CV_RGB(0,250,0)) const;
};

