/**
 *
 * @file fire_recognition.h
 *
 * @brief process video information that raspberry pi gets
 *
 * Functions for recognize fire target
 *
 * @author XU Peter, <peterxu9710@outlook.com>
 *         CUI Yuxiang
 *         WANG Xin
 *
 */


// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------


#include "opencv2/highgui.hpp"
#include "opencv/cv.h"
#include "opencv/cvaux.h"
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"

using namespace cv;
using namespace std;

class firedetect{

public:

    firedetect();
    firedetect(Mat inputImg);
    firedetect(Mat inputImg, int orig_RGB[3]);
    void getfilename();
    void read(Mat newimg);
    void CheckColor();
    void DrawFire(Mat fireImgout);
    float get_desx();
    float get_desy();
    double get_square();
    Mat getoutput();
    ~firedetect();
    bool firedetected();


private:

    double square;
    char* inImg;
    Mat inputfireimage;
    Point2f destination;
    int RGB[3];
//    Mat output;

};
