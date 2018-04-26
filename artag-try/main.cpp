//
// Created by 崔瑜翔 on 2018/4/3.
//

#include <opencv2/opencv.hpp>
#include <GL/glut.h>
#include "MarkerDetector.h"

using namespace cv;
using namespace std;

static GLint imagewidth;
static GLint imageheight;
static GLint pixellength;
static GLubyte* pixeldata;
#define GL_BGR_EXT 0x80E0

Matrix44 projectionMatrix;
vector<Marker> m_detectedMarkers;
//GLuint defaultFramebuffer, colorRenderbuffer;

MarkerDetector markerDetector;

int cameraNumber = 0;

Mat_<float>  camMatrix;
Mat_<float>  distCoeff;

//void initCamera(VideoCapture &VideoCapture,int cameraNumber);
void readCameraParameter();
void readCameraParameter1();

void build_projection(Mat_<float> cameraMatrix)
{
    float near = 0.01;  // Near clipping distance
    float far = 100;  // Far clipping distance


    float f_x = cameraMatrix(0,0); // Focal length in x axis
    float f_y = cameraMatrix(1,1); // Focal length in y axis (usually the same?)
    float c_x = cameraMatrix(0,2); // Camera primary point x
    float c_y = cameraMatrix(1,2); // Camera primary point y

    projectionMatrix.data[0] =  - 2.0 * f_x / imagewidth;
    projectionMatrix.data[1] = 0.0;
    projectionMatrix.data[2] = 0.0;
    projectionMatrix.data[3] = 0.0;

    projectionMatrix.data[4] = 0.0;
    projectionMatrix.data[5] = 2.0 * f_y / imageheight;
    projectionMatrix.data[6] = 0.0;
    projectionMatrix.data[7] = 0.0;

    projectionMatrix.data[8] = 2.0 * c_x / imagewidth - 1.0;
    projectionMatrix.data[9] = 2.0 * c_y / imageheight - 1.0;
    projectionMatrix.data[10] = -( far+near ) / ( far - near );
    projectionMatrix.data[11] = -1.0;

    projectionMatrix.data[12] = 0.0;
    projectionMatrix.data[13] = 0.0;
    projectionMatrix.data[14] = -2.0 * far * near / ( far - near );
    projectionMatrix.data[15] = 0.0;
}

void setMarker(const vector<Marker>& detectedMarkers)
{
    m_detectedMarkers = detectedMarkers;
}




int main(int argc,char *argv[])
{
    Mat src,dst;
    Mat grayscale,threshImg;
    vector< vector<Point> > contours;
    vector<Marker> markers;

    //设置摄像头或视频
   // if(argc > 1)
    //    cameraNumber = atoi(argv[1]);

    //打开摄像头，检测是否打开
  //  VideoCapture camera;
   // initCamera(camera,cameraNumber);
    //设置窗口长宽比
   // camera.set(CV_CAP_PROP_FRAME_WIDTH,640);
   // camera.set(CV_CAP_PROP_FRAME_HEIGHT,480);

    //导入相机内参数
    readCameraParameter1();
//    cout<<"/*cammatrix"<<endl;
//    cout << camMatrix << endl;
//    cout<<"cammatrix*/"<<endl;
    //while(true)
    //{
        //camera >> src;
        src = imread("/Users/cuiyuxiang/Desktop/try.jpg");
        if(src.empty()){
            cerr << "ERROR: could not grab a camera frame!" << endl;
            exit(1);
        }

        markerDetector.processFrame(src,camMatrix, distCoeff,markers);

        imwrite("test.bmp",src);


        char keypress =  waitKey(0);
        if(keypress == 27){
            //break;
            exit(1);
        }
    //}
    return 0;
}

//void initCamera(VideoCapture &VideoCapture, int cameraNumber)
//{
//  VideoCapture.open(cameraNumber);
//  if(!VideoCapture.isOpened()){
//      cerr << "ERROR: Could not access the camera!!!" << endl;
//      exit(1);
//  }
 // cout << "Loaded camera" << cameraNumber << "." << endl;
//}

void readCameraParameter()
{
    camMatrix = Mat::eye(3, 3, CV_64F);
    distCoeff = Mat::zeros(8, 1, CV_64F);

    FileStorage fs("/home/zhu/program_c/opencv_study/AR_CV_GL/mastering_opencv/marker_AR/camera_calibration/out_camera_data.yml",FileStorage::READ);
    if (!fs.isOpened())
    {
        cout << "Could not open the configuration file!" << endl;
        exit(1);
    }
    fs["Camera_Matrix"] >> camMatrix;
    fs["Distortion_Coefficients"] >> distCoeff;
    fs.release();
    cout << camMatrix << endl;
    cout << distCoeff << endl;
}

void readCameraParameter1()
{

    camMatrix = Mat::eye(3, 3, CV_64F);
    distCoeff = Mat::zeros(8, 1, CV_64F);

    camMatrix(0,0) = 6.24860291e+02 * (640./352.);
    camMatrix(1,1) = 6.24860291e+02 * (480./288.);
    camMatrix(0,2) = 640 * 0.5f; //640
    camMatrix(1,2) = 480 * 0.5f; //480,我改的！牛逼不？！

    for (int i=0; i<4; i++)
        distCoeff(i,0) = 0;
}






