//
// Created by 崔瑜翔 on 2018/3/20.
//

#ifndef MARKER_AR_MARKERDETECTOR_H
#define MARKER_AR_MARKERDETECTOR_H
#include <vector>
#include <iostream>
#include <string>
#include <sstream>
#include <cstddef> //c++头文件，包含一些重定义的类型，size_t
#include <opencv2/opencv.hpp>

#include "Marker.h"

using namespace cv;
using namespace std;

//定义一个标记检测类：MarkerDetector
class MarkerDetector
{
public:
    typedef vector<Point> PointsVector;
    typedef vector<PointsVector> ContoursVector;

    MarkerDetector();

    void processFrame(const Mat& frame,Mat_<float>& camMatrix,Mat_<float>& distCoeff,vector<Marker>& markers);

    const vector<Transformation>& getTransformations() const;

public:
    void prepareImage(const Mat& src,Mat& grayscale);
    void performThreshold(const Mat& grayscale,Mat& thresholdImg);
    void findContour(cv::Mat& thresholdImg, ContoursVector& contours, int minContourPointsAllowed) const;
    void findCandidates(const ContoursVector& contours,vector<Marker>& detectedMarkers);

    void recognizeMarkers(const Mat& grayscale,vector<Marker>& detectedMarkers);
    void estimatePosition(vector<Marker>& detectedMarkers,Mat_<float>& camMatrix,Mat_<float>& distCoeff);
    bool findMarkers(const Mat& frame,vector<Marker>& detectedMarkers);

public:

    Size markerSize;

    ContoursVector m_contours;
    vector<Point3f> m_markerCorners3d;
    vector<Point2f> m_markerCorners2d;

    Mat m_grayscaleImage;
    Mat m_thresholdImg;
    Mat canonicalMarkerImage;

    vector<Transformation> m_transformations;

    float m_minContourLengthAllowed;
};

struct BGRAVideoFrame
{
    size_t width;
    size_t height;
    size_t stride;

    unsigned char * data;
};

template <typename T>
string ToString(const T& value)
{
    ostringstream stream;
    stream << value;
    return stream.str();
}




#endif //MARKER_AR_MARKERDETECTOR_H
