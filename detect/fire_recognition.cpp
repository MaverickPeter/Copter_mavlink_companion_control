#include "highgui.h"
#include "cv.h"
#include <cvaux.h>
#include<opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "fire_recognition.h"
using namespace cv;
using namespace std;

//int redThre = 120; // 115~135
//int saturationTh = 55; //55~65

bool firedetect::firedetected()
{
    float square0 = 640*480;
    if(square/square0 != 0 )
    {
        return 1;
    }
    return 0;
}

double firedetect::get_square(){
	return square;
}

float firedetect::get_desx() {
	return destination.x;
}

float firedetect::get_desy() {
	return destination.y;
}

firedetect::firedetect() {
	inImg = new char[100];
}//构造函数
firedetect::firedetect(Mat inputImg) {
	inputfireimage = inputImg;
        destination.x = 0;
        destination.y = 0;
        square = 0;
}//复制构造函数
firedetect::firedetect(Mat inputImg, int origs_RGB[3]) {
    inputfireimage = inputImg;
    destination.x = 0;
    destination.y = 0;
    square = 0;
    RGB[0] = origs_RGB[0];
    RGB[1] = origs_RGB[1];
    RGB[2] = origs_RGB[2];
}//根据文件名构造
firedetect::~firedetect() {
//	delete[]inImg;
}

void firedetect::getfilename() {
	cin >> inImg;
}//获取文件名（好像没什么用）

void firedetect::CheckColor()
{
	Mat fireImg;
	fireImg.create(inputfireimage.size(), CV_8UC1);

	Mat fireImgout;
	Mat fireImgout0;
    Mat multiRGB[3];

	split(inputfireimage, multiRGB);

	for (int i = 0; i < inputfireimage.rows; i++)
	{
		for (int j = 0; j < inputfireimage.cols; j++)
		{
			float B, G, R;
			B = multiRGB[0].at<uchar>(i, j); //每个像素的R,G,B值
			G = multiRGB[1].at<uchar>(i, j);
			R = multiRGB[2].at<uchar>(i, j);

//			float minValue = min(min(B, G), R);
			//S分量的计算
//			double S = (1 - 3.0*minValue / (R + G + B));

			//火焰判断条件
//			if (R > redThre &&R >= G && G >= B && S >((255 - R) * saturationTh / redThre))
            if (R > (int)RGB[0]-20 && R < (int)RGB[0]+20 && G < (int)RGB[1]+20 && G > (int)RGB[1]-20 && B < (int)RGB[2]+20 && B > (int)RGB[2]+20)
			{
				fireImg.at<uchar>(i, j) = 255;
            }
			else
			{
				fireImg.at<uchar>(i, j) = 0;
			}
		}
	}

	Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));

	dilate(fireImg, fireImgout, element);
//	cout<<"Before DrawFire()"<<endl;
	DrawFire(fireImgout);//实际用的

						 //    cvtColor(inputfireimage, inputfireimage, CV_BGR2GRAY );
						 //    imshow("showFire",inputfireimage);
}

void firedetect::DrawFire(Mat fireImgout)
{
	vector< vector<Point> > contours_set;//保存轮廓
	Mat gray;
	//  cvtColor(fireImgout, gray, CV_BGR2GRAY );

	findContours(fireImgout, contours_set, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	vector<Rect> boundRect(contours_set.size());
	vector<Point> firepoint(contours_set.size());
	Mat result0;
	Scalar holeColor;
	Scalar externalColor;
	int count = 0;
	vector<vector<Point> >::iterator iter = contours_set.begin();
	//    if (contours_set.size() == 0) {
	//        Point a,b;
	//        a.x = 100;
	//        a.y = 100;
	//        b.x = 400;
	//        b.y = 400;
	//        rectangle(inputfireimage,a,b,Scalar(0,255,0));
	//    }
	for (; iter != contours_set.end(); )
	{

		boundRect[count] = boundingRect(*iter);

        if (boundRect[count].area()> 500)
		{
			firepoint[count].x = (boundRect[count].tl().x + boundRect[count].br().x) / 2;
			firepoint[count].y = (boundRect[count].tl().y + boundRect[count].br().y) / 2;
			//保存火焰点
                        square -= (boundRect[count].br().x-boundRect[count].tl().x)*(boundRect[count].tl().y-boundRect[count].br().y);
			rectangle(inputfireimage, boundRect[count].tl(), boundRect[count].br(), Scalar(0, 255, 0));
			circle(inputfireimage, firepoint[count], 5, Scalar(0, 255, 0));
			rectangle(inputfireimage, boundRect[count], Scalar(0, 255, 0));
			++iter;
			count++;
		}
		else
		{
			iter = contours_set.erase(iter);
		}

	}

//	cout<<"After get center"<<endl;
//	float radius;
//	Point2f center;
//	if (count > 0) {
//		minEnclosingCircle(firepoint, center, radius);
//		circle(inputfireimage, center, 5, Scalar(255, 0, 0));
//	}

//	destination = center;

	float a = 0;
	float b = 0;
	if (count>0) {
		for (int i = 0; i < count; i++) {
			a += firepoint[i].x;
			b += firepoint[i].y;
		}
		a = a / count;
		b = b / count;


		destination.x = a;
		destination.y = b;
	}
	else {
		destination.x = 0;
		destination.y = 0;
	} 
	circle(inputfireimage, destination, 5, Scalar(255, 0, 0));


	//    Point a,b;
	//    a.x = 100;
	//    a.y = 100;
	//    b.x = 200;
	//    b.y = 200;
	//
	//    rectangle(inputfireimage,a,b,Scalar(0,255,0));
	waitKey(1);

}

Mat firedetect::getoutput() {
	return inputfireimage;
}

void firedetect::read(Mat newimg) {
	inputfireimage = newimg;
}

/*
int main() {


	VideoCapture capture(0);//如果是笔记本，0打开的是自带的摄像头，1 打开外接的相机
							//    double rate = 25.0;//视频的帧率
							//    Size videoSize(1280,960);
							//视频写入对象  
	//写入视频文件名  
	double rate =25; //capture.get(CV_CAP_PROP_FPS);

	Size videoSize(capture.get(CV_CAP_PROP_FRAME_WIDTH),

		capture.get(CV_CAP_PROP_FRAME_HEIGHT));
	VideoWriter writer;
	writer.open("/home/pi/result.avi", CV_FOURCC('P', 'I', 'M', '1'), rate, videoSize);

	Mat frame;
	Mat fireoutput;
//	cout << "success in opening camera"<<endl;
	int num = 0;
	while (capture.isOpened())
	{
		
		num++;
		capture >> frame;
		if (frame.empty())
			break;
		firedetect first(frame);
//		cout<<"success in firedetect"<<endl;
		writer << frame;
		first.CheckColor();
//		cout<<"finish CheckColor()"<<endl;
		cout << "(" << first.getdes_x() <<"."<< first.getdes_y() <<")" << endl;
//
		//       writer << fireoutput;

		if (waitKey(10) == 27)//27是键盘摁下esc时，计算机接收到的ascii码值
		{
			break;
		}

	}
	capture.release();

	return 0;
}
*/

