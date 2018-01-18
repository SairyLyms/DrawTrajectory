#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <complex>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
template<class T, size_t N> size_t countof(const T (&array)[N]) { return N; }
using namespace cv;

int DrawLines(void);
void DrawClothoid(float x0,float y0,float phi0,float h,float phiV,float phiU,int8_t n,Mat *image);
void ImageOverray(Mat plotImage,Mat camImage,Mat *outImage);

float pi()
{
  return 3.141593f;
}

int main(int argc, char *argv[])
{
    CvSize imageSize = cvSize(200, 200);
    //Mat plotImage = Mat::zeros(imageSize,CV_8UC3);
    Mat plotImage(imageSize,CV_8UC3,Scalar(255,255,255));//デバッグ用に白いVer.
    //カメラ検出
    VideoCapture cap(0);

    //カメラ検出できない場合、エラーで終了
//    if(!cap.isOpened())
//    {
//        return -1;
//    }
    //車両情報受信プログラム(bluetooth通信)
    //if車両からクロソイドパラメータ受信した場合
    /*作成する*/
        //plotImageにOverRay用クロソイド曲線描画
        DrawClothoid(0.0f,0.0f,0.0f,10.0f,0.0f,0.0f,10,&plotImage);

    //else if車両が走行中の場合
        //車両挙動に合わせてplotImageを平行移動・回転させる

    //カメラ画像読み込み
    Mat camImage(Size(640, 480), CV_8UC3, Scalar(255,0,0));

    //Overray画像作成プログラム
    Mat overrayImage;
    ImageOverray(plotImage,camImage,&overrayImage);

    namedWindow("drawing", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
    imshow("drawing", overrayImage);
    waitKey(0);

    return 0;
}



std::complex<float> Slope(float phi0,float phiV, float phiU, float S)
{
    std::complex<float> i(0,phi0 + phiV * S + phiU * S * S);
    return  std::exp(i);
}

void DrawClothoid(float x0,float y0,float phi0,float h,float phiV,float phiU,int8_t n,Mat *image)
{
    //車両から取得した軌跡情報による描画
    Point clothoidPt[10] = {};

  std::complex<float> integral(0,0);
  float w = 1/(float)n;
  float s = 0.0f,cv0 = 0.0f,cv1 = 0.0f,x = 0.0f,y = 0.0f;
  std::complex<float>half(0.5f,0.0f),wcpx(w,0.0f);

  for (int8_t i=0; i<n; i++){
    //積分(台形法)
    integral += (Slope(phi0, phiV, phiU, s) + Slope(phi0, phiV, phiU, s+w)) * half * wcpx;
    s += w;

    x = h * std::abs(integral) * std::cos(std::arg(integral)) + x0;
    y = h * std::abs(integral) * std::sin(std::arg(integral)) + y0;

    std::cout << "x,"<< x << "y," << y << std::endl;
    std::cout << "arg,"<< std::arg(integral) << std::endl;
    std::cout << "integral,"<< integral << std::endl;

    //描画関数
    float xRend = -x + 100,yRend = -y + 100;
    clothoidPt[i] = Point(yRend,xRend);//Clothoid x,y rendering
    circle (*image, clothoidPt[i], 5, cv::Scalar(200,100,0), -1, CV_AA);
  }
}

void ImageOverray(Mat plotImage,Mat camImage,Mat *outImage)
{
    //http://www.mdpi.com/1424-8220/12/4/4431/htm
    //I-TVTM変換する
    //http://robotex.ing.ee/2012/01/coordinate-mapping/

    Mat plotImageExpd;
    Size sizePltImgExpd(camImage.cols,camImage.cols);
    resize(plotImage,plotImageExpd,sizePltImgExpd);

    // 1  3
    // 2  4 の順番
    const Point2f src_pt[] = {  Point2f(             0.0f ,                     0.0f ),
                                Point2f(             0.0f , 0.5f * plotImageExpd.rows),
                                Point2f(plotImageExpd.cols,                     0.0f ),
                                Point2f(plotImageExpd.cols, 0.5f * plotImageExpd.rows)  };

    const Point2f dst_pt[] = {  Point2f( 0.0f * plotImageExpd.cols , 0.5f * plotImageExpd.rows),
                                Point2f(-1.0f * plotImageExpd.cols ,        plotImageExpd.rows),
                                Point2f( 1.0f * plotImageExpd.cols , 0.5f * plotImageExpd.rows),
                                Point2f( 2.0f * plotImageExpd.cols ,        plotImageExpd.rows)  };

    //Homography 行列を計算
    const Mat hom_matrix = getPerspectiveTransform(src_pt,dst_pt);
    warpPerspective(plotImageExpd, plotImageExpd, hom_matrix, sizePltImgExpd);

    *outImage = plotImageExpd;

}
