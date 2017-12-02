#ifndef HISTOGRAM1D_H
#define HISTOGRAM1D_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;

class Histogram1D{
private:
    int histSize[1];       //项的数量
    float hranges[2];   //像素的最小及最大值
    const float* ranges[1];
    int channels[1];    //仅用到一个通道
public:
    Histogram1D(){
        //准备1D直方图的参数
        histSize[0] =256;
        hranges[0] = 0.0;
        hranges[1] = 255.0;
        ranges[0]  = hranges;
        channels[0] = 0;  //默认情况，我们考察0号通道
    }

    //计算1D直方图
    MatND getHistogram(const Mat&image)
    {
        MatND  hist;
        //计算直方图
        calcHist(&image,
                 1,                     //计算单张图像的直方图
                 channels,        //通道的数量
                 Mat(),              //不使用图像作为掩码
                 hist,                //返回的直方图
                 1,                    //这是1D的直方图
                 histSize,         //项的数量
                 ranges            //像素值的范围
                 );
        return hist;
    }

    //计算1D直方图，并返回一幅图像
    Mat getHistogramImage(const Mat &image)
    {
        //首先计算直方图
        MatND hist = getHistogram(image);
        //获取最大值和最小值
        double maxVal = 0;
        double minVal = 0;
        minMaxLoc(hist,&minVal,&maxVal,0,0);
        //显示直方图的图像
        Mat histImg(histSize[0],histSize[0],CV_8U,Scalar(255));
        //设置最高点为nbins的90%
        int hpt  = static_cast<int>(0.9*histSize[0]);
        //每个条目都绘制一条垂直线
        for(int h = 0;h<histSize[0];h++){
            float binVal =hist.at<float>(h);
            int intensity = static_cast<int>(binVal*hpt/maxVal);
            //两点之间绘制一条线
            line(histImg,Point(h,histSize[0]),
                    Point(h,histSize[0]-intensity),
                    Scalar::all(0));
        }
        return histImg;
    }
};
#endif // HISTOGRAM1D_H
