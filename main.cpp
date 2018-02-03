#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <iostream>
#include <complex>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <sys/socket.h>
#include "BTComm.h"
template<class T, size_t N> size_t countof(const T (&array)[N]) { return N; }
using namespace cv;

int DrawLines(void);
void DrawClothoid(float x0,float y0,float phi0,float h,float phiV,float phiU,int8_t n,Mat *image);
void DrawClothoidSimple(float h,float phiV,float phiU,float odo,int8_t n,Mat *image);
void ImageOverray(Mat plotImage,Mat camImage,Mat *outImage);
float pi()
{
  return 3.141593f;
}

int main(int argc, char *argv[])
{
    SetupNcurses();
    OpenBT(&sockBT);
    CvSize imageSize = cvSize(200, 200);
    //カメラ検出
    //VideoCapture cap(0);
    //カメラ検出できない場合、エラーで終了
//    if(!cap.isOpened())
//    {
//        return -1;
//    }
    namedWindow("drawing", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
    while(1){
        erase();
        ReadBT(&sockBT);
        SendCommandToVehicle(stateMode,&sockBT);//車両状態・キーボード入力に応じてコマンド送信
        #if 1
        Mat plotImage(imageSize,CV_8UC3,Scalar(0,0,0));//デバッグ用に白いVer.
        //カメラ画像読み込み
        //Mat camImage(Size(640, 640), CV_8UC3, Scalar(255,200,200));
        Mat camImage = imread("road.png");
        //plotImageにOverRay用クロソイド曲線描画
        //DrawClothoid(0.0f,0.0f,yawAngle,100.0f,0.0f,0.0f,10,&plotImage);
        //DrawClothoidSimple(100.0f,0.0f,0.0f,0.0f,10,&plotImage);
        DrawClothoidSimple(h * 10.0f,phiV,phiU,odo * 10.0f,10,&plotImage);
        //Overray画像作成プログラム
        Mat overrayImage;
        ImageOverray(plotImage,camImage,&overrayImage);
        imshow("drawing", overrayImage);
        waitKey(1);
        refresh();
        #endif
    }
    CloseBT(&sockBT);
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
#if 0
    std::cout << "x,"<< x << "y," << y << std::endl;
    std::cout << "arg,"<< std::arg(integral) << std::endl;
    std::cout << "integral,"<< integral << std::endl;
#endif
    //描画関数
    float xRend = -x + 100,yRend = -y + 100;
    clothoidPt[i] = Point(yRend,xRend);//Clothoid x,y rendering
    circle (*image, clothoidPt[i], 1, cv::Scalar(200,180,0), -1, CV_AA);
  }
}

void DrawClothoidSimple(float h,float phiV,float phiU,float odo,int8_t n,Mat *image)
{
    //車両から取得した軌跡情報による描画
    Point clothoidPt[10] = {};

    std::complex<float> integral(0,0);
    float odoNorm = 0.0f;
    float w = 0.0f;
    float s = 0.0f,x = 0.0f,y = 0.0f;
    std::complex<float>half(0.5f,0.0f),wcpx(w,0.0f);

    if(odo > h){
        odoNorm = 1.0f;
        }
    else if(odo < 0){
        odoNorm = 0.0f;
        }
    else{
        odoNorm = odo/h;
    }
    w = (1.0f - odoNorm) / n;
    wcpx = std::complex<float>(w,0.0f);
    for (int8_t i=0; i<n; i++){
        //積分(台形法)
        integral += (Slope(0, phiV, phiU, s) + Slope(0, phiV, phiU, s+w)) * half * wcpx;
        s += w;

        x = h * std::abs(integral) * std::cos(std::arg(integral));
        y = h * std::abs(integral) * std::sin(std::arg(integral));

        //描画関数
        float xRend = -x + 100,yRend = -y + 100;
        clothoidPt[i] = Point(yRend,xRend);//Clothoid x,y rendering
        circle (*image, clothoidPt[i], 5, cv::Scalar(200,180,0), -1, CV_AA);
  }
}

void ImageOverray(Mat plotImage,Mat camImage,Mat *outImage)
{
    //http://www.mdpi.com/1424-8220/12/4/4431/htm
    //I-TVTM変換する
    //http://robotex.ing.ee/2012/01/coordinate-mapping/

    //アルファブレンディング参考
    //http://catalina1344.hatenablog.jp/entry/2014/05/07/210321
    //https://www.learnopencv.com/alpha-blending-using-opencv-cpp-python/

    int maxVal = pow(2, 8*camImage.elemSize1())-1;
    Size sizePltImgExpd(camImage.rows,camImage.cols);
    Mat plotImageExpd,plotImageExpdAlpha,plotImageDst_rgb,plotImageDst_Alpha,camImageDst_Alpha;

    //チャンネル情報ベクタ
    std::vector<Mat>plotImage_rgba,plotImage_rgb, plotImage_alpha,camImage_alpha;

    resize(plotImage,plotImageExpd,sizePltImgExpd);
    //cvtColor(camImage, camImage, CV_RGB2RGBA);//アルファチャンネル化
    cvtColor(plotImageExpd, plotImageExpd, CV_RGB2RGBA);//アルファチャンネル化

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

    //Homography 行列を計算し変形
    const Mat hom_matrix = getPerspectiveTransform(src_pt,dst_pt);
    warpPerspective(plotImageExpd, plotImageExpd, hom_matrix, sizePltImgExpd);

    plotImageExpdAlpha = plotImageExpd.clone();

    //黒部分を透明化したマスク画像作成
    for (int y = 0; y < plotImageExpdAlpha.rows; ++y) {
        for (int x = 0; x < plotImageExpdAlpha.cols; ++x) {
            cv::Vec4b px = plotImageExpdAlpha.at<Vec4b>(x, y);
            if (px[0] + px[1] + px[2] == 0) {
                px[3] = 0;
                plotImageExpdAlpha.at<Vec4b>(x, y) = px;
            }
            //std::cout << "x," << x << "y," << y << "Alp," << px << std::endl;
        }
    }

    split(plotImageExpdAlpha, plotImage_rgba);
    for(int8_t i=0;i<3;i++){
        plotImage_rgb.push_back(plotImage_rgba[i]);
        plotImage_alpha.push_back(plotImage_rgba[3]);
        camImage_alpha.push_back(maxVal-plotImage_rgba[3]);
    }

    merge(plotImage_rgb, plotImageDst_rgb);
    merge(plotImage_alpha, plotImageDst_Alpha);
    merge(camImage_alpha, camImageDst_Alpha);

    *outImage = plotImageDst_rgb.mul(plotImageDst_Alpha,1.0/(double)maxVal) + camImage.mul(camImageDst_Alpha,1.0f/(double)maxVal);
}