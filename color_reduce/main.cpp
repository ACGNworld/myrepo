#include <iostream>
#include "opencv2/opencv.hpp"
using namespace cv;
using namespace std;

void colorreduce(Mat& inputimg,Mat& outputimg,int div);

int main() {
    Mat img = imread("/home/fjj/图片/wanye.jpg");
    resize(img,img,Size(1024,576));
    Mat dst;
    dst.create(img.rows,img.cols,img.type());
    double time0 = static_cast<double>(getTickCount());
    colorreduce(img,dst,64);
    time0 = ((double)getTickCount()-time0)/getTickFrequency();
    cout<<time0<<endl;
    imshow("picture",dst);
    waitKey(0);
    return 0;
}


void colorreduce(Mat& inputimg,Mat& outputimg,int div)
{
    outputimg = inputimg.clone();
    int rownumber = outputimg.rows;
    int colnumber = outputimg.cols;
    for(int i = 0;i < rownumber;i++)
        for(int j = 0;j< colnumber;j++)
        {
            outputimg.at<Vec3b>(i,j)[0]=outputimg.at<Vec3b>(i,j)[0]/div*div+div/2;
            outputimg.at<Vec3b>(i,j)[1]=outputimg.at<Vec3b>(i,j)[1]/div*div+div/2;
            outputimg.at<Vec3b>(i,j)[2]=outputimg.at<Vec3b>(i,j)[2]/div*div+div/2;
        }
    /*int colnumber = inputimg.cols*outputimg.channels();
    for(int i = 0;i < rownumber;i++)
    {
        uchar*data = outputimg.ptr<uchar>(i);  //指针访问，快
        for(int j = 0;j < colnumber;j++)
        {data[j] = data[j]/div*div+div/2;}
    }*/
}