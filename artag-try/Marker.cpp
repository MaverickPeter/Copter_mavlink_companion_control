//
// Created by 崔瑜翔 on 2018/3/20.
//

#include "Marker.h"

using namespace cv;
using namespace std;

Marker::Marker()
        :id(-1)
{

}

bool operator<(const Marker& M1,const Marker& M2)
{
    return M1.id < M2.id;
}

Mat Marker::rotate(Mat in)//旋转矩阵90度
{
    Mat out;
    in.copyTo(out);
    for(int i=0;i<in.rows;i++)
    {
        for(int j=0;j<in.cols;j++)
        {
            out.at<uchar>(i,j)=in.at<uchar>(in.cols-j-1,i);//at<uchar>用来指定某个位置的像素，同时指定数据类型。就是交换元素，怎么交换的？
        }
    }
    return out;
}


int Marker::hammDistMarker(Mat bits)//计算海明距离
{
    int ids[4][5]=
            {
                    {1,0,0,0,0},
                    {1,0,1,1,1},
                    {0,1,0,0,1},
                    {0,1,1,1,0}
            };

    int dist = 0;

    for(int y=0;y<5;y++)
    {
        int minSum = 1e5;//设定海明距离初始值

        for(int p=0;p<4;p++)
        {
            int sum=0;

            for(int x=0;x<5;x++)
            {
                sum += bits.at<uchar>(y,x) == ids[p][x]?0:1;
            }
            if(minSum>sum)
                minSum=sum;
        }//找到待定marker中和已知的四行中海明距离最小的一行，也即匹配的最好的一行

        dist += minSum;//得到整个待定marker的海明距离，如果等于零则代表这个待定marker每一行都符合规定
    }

    return dist;
}

int Marker::mat2id(const Mat& bits)//移位，求或，再移位，得到最终的ID
{
    int val=0;
    for(int y=0;y<5;y++)
    {
        val<<=1;//移位操作
        if(bits.at<uchar>(y,1)) val |= 1;
        val<<=1;
        if(bits.at<uchar>(y,3)) val |= 1;
    }
    return val;
}//得到marker对应的ID

int Marker::getMarkerId(Mat& markerImage,int &nRotations)//判断是不是标准marker，并计算对应的ID
{
    assert(markerImage.rows == markerImage.cols);
    assert(markerImage.type() == CV_8UC1);

    Mat grey = markerImage;

    threshold(grey,grey,125,255,THRESH_BINARY | THRESH_OTSU);//对候选标记区域的灰度图使用大律OSTU算法，求取二值化图，大范围图片用这个算法会影响性能。

#ifdef SHOW_DEBUG_IMAGES
    imshow("Binary marker",grey);
  imwrite("Binary marker" + ".png",grey);
#endif


    int cellSize = markerImage.rows/7;//检查单元的大小

    for(int y=0;y<7;y++)
    {
        int inc = 6;

        if(y == 0 || y == 6) inc=1;//如果是第一行和最后一行，检查整条

        for(int x=0;x<7;x+=inc)
        {
            int cellX = x*cellSize;
            int cellY = y*cellSize;
            Mat cell = grey(Rect(cellX,cellY,cellSize,cellSize));

            int nZ = countNonZero(cell);

            if(nZ > (cellSize*cellSize)/2)
            {
                return -1;//边框白的，图像不是marker
            }
        }
    }//检查边框是不是一个artag的边框

    Mat bitMatrix = Mat::zeros(5,5,CV_8UC1);

    //得到信息，黑白块分布，变成一个5x5矩阵
    for(int y=0;y<5;y++)
    {
        for(int x=0;x<5;x++)
        {
            int cellX = (x+1)*cellSize;
            int cellY = (y+1)*cellSize;
            Mat cell = grey(Rect(cellX,cellY,cellSize,cellSize));

            int nZ = countNonZero(cell);
            if(nZ > (cellSize*cellSize)/2)
                bitMatrix.at<uchar>(y,x) = 1;
        }
    }

    //检查所有的旋转
    Mat rotations[4];
    int distances[4];

    rotations[0] = bitMatrix;
    distances[0] = hammDistMarker(rotations[0]);//求没有旋转的矩阵的海明距离。

    pair<int,int> minDist(distances[0],0);//把求得的海明距离和旋转角度作为最小初始值对，每个pair都有两个属性值first和second

    for(int i=1;i<4;i++)//一次次旋转，判断这个矩阵与参考矩阵旋转多少度
    {
        //计算最近的可能元素的海明距离
        rotations[i] = rotate(rotations[i-1]);//每次旋转90度
        distances[i] = hammDistMarker(rotations[i]);

        if(distances[i] < minDist.first)
        {
            minDist.first = distances[i];
            minDist.second = i;//这个pair的第二个值是代表旋转几次，每次90度。
        }
    }

    nRotations = minDist.second;//这个是将返回的旋转角度值
    if(minDist.first == 0)//若海明距离为0,则根据这个旋转后的矩阵计算标识ID
    {
        return mat2id(rotations[minDist.second]);
    }

    return -1;
}

void Marker::drawContour(Mat& image,Scalar color) const//在图像上画线，描绘出轮廓。
{
    float thickness = 2;

    line(image,points[0],points[1],color,thickness,CV_AA);
    line(image,points[1],points[2],color,thickness,CV_AA);
    line(image,points[2],points[3],color,thickness,CV_AA);//thickness线宽
    line(image,points[3],points[0],color,thickness,CV_AA);//CV_AA是抗锯齿
}
