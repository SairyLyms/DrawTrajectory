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
void DrawClothoid(float x0,float y0,float phi0,float h,float phiV,float phiU,float* x,float* y,int8_t n);

float pi()
{
  return 3.141593f;
}

int main(int argc, char *argv[])
{
    VideoCapture cap(0);
//    if(!cap.isOpened())
//    {
//        return -1;
//    }
    DrawClothoid(0,0,-1.57f,100.0f,0,2,NULL,NULL,10);
    //DrawLines();
#if 0
    Mat img;
    while(1){
        cap >> img;
        if(img.empty())
            return -1;
        ellipse(img,Point(0.5 * img.size().width - 100,0.5 * img.size().height),Size(100,100),0,0,-90,Scalar( 255, 127, 0 ),5);//描画関数入れる
        resize(img, img, Size(), 0.5, 0.5);
        namedWindow( "video", CV_WINDOW_AUTOSIZE );
        imshow("video", img);
        waitKey(1);
        }

#endif
    return 0;
}



int DrawLines(void)
{
//車両から取得した軌跡情報による描画
CvSize imageSize = cvSize(200, 200);

Mat plotImage = Mat::zeros(imageSize,CV_8UC3);
Point clothoidPt[10];

for(uint8_t i=0;i<10;i++){
    float x = i*10,y = i*0;
    float xRend = -x + 100,yRend = -y + 100;
    clothoidPt[i] = Point(yRend,xRend);//Clothoid x,y rendering
    circle(plotImage, clothoidPt[i], 5, cv::Scalar(200,100,0), -1, CV_AA);
}

namedWindow("drawing", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
imshow("drawing", plotImage);

#if 0
      CvSize imageSize = cvSize(640, 480);
      Mat camImage = imread("./image.jpg");
      //Mat camImage(imageSize, CV_8UC3,Scalar(200,0,0));
      Mat poly(cvSize(200, 200), CV_8UC3,Scalar(200,200,200));
      int matSizeX = poly.size().width,matSizeY = poly.size().height;
      Point point3[][2] =
      {
         { Point(0, 0),        Point(matSizeX, 0) },
         { Point(matSizeX, 0), Point(matSizeX,matSizeY) },
         { Point(0 ,0),        Point( 0,matSizeY) },
         { Point(0 ,matSizeY), Point(matSizeX,matSizeY) },
      };

      const Point *pts3[] = { point3[0], point3[1], point3[2], point3[3] };

      int npts3[] = { 2, 2, 2 ,2};

      polylines(poly,
         pts3,
         npts3,
         countof(npts3),
         false,
         Scalar( 0, 0, 255),
         3,
         LINE_8
         );

    //Convert to perspective image
    Mat mask;
    Point2f pts1[] = {Point2f(0,0),Point2f(0,matSizeY),Point2f(matSizeX,matSizeY),Point2f(matSizeX,0)};
    Point2f pts2[] = {Point2f(0,0.5 * matSizeY),Point2f(-matSizeX,matSizeY),Point2f(2 * matSizeX,matSizeY),Point2f(matSizeX,0.5*matSizeY)};
    Mat perspective_matrix = getPerspectiveTransform(pts1, pts2);
    warpPerspective(poly, poly, perspective_matrix, poly.size(), INTER_LINEAR);
    resize(poly,poly,imageSize);
    inRange(poly, Scalar(0, 0, 0, 0), Scalar(0, 0, 0, 0), mask);
    mask = 255 - mask;
    Mat brend(camImage.clone());
    poly.copyTo(brend, mask);

    namedWindow("polylines");
    imshow("polylines", poly);

    namedWindow("Perspective");
    imshow("Perspective", brend);
#endif

    waitKey(0);
}


std::complex<float> Slope(float phi0,float phiV, float phiU, float S)
{
    std::complex<float> i(0,phi0 + phiV * S + phiU * S * S);
    return  std::exp(i);
}


void DrawClothoid(float x0,float y0,float phi0,float h,float phiV,float phiU,float* xpt,float* ypt,int8_t n)
{
    //車両から取得した軌跡情報による描画
    CvSize imageSize = cvSize(200, 200);
    Mat plotImage = Mat::zeros(imageSize,CV_8UC3);
    Point clothoidPt[10];
    //クロソイドパラメータから曲線描画
    //DrawClothoid(0,0,0,10,0,2.0,NULL,NULL,10);

  std::complex<float> integral(0,0);
  float w = 1/(float)n;
  float s,cv0,cv1,x,y;
  std::complex<float>half(0.5f,0.0f),wcpx(w,0.0f);
  static float lengthTotal,velocity;

  for (int8_t i=0; i<n; i++){
    integral += (Slope(phi0, phiV, phiU, s) + Slope(phi0, phiV, phiU, s+w)) * half * wcpx;
    //integral.real() *= 0.5 * w;
    s += w;

    x = h * std::abs(integral) * std::cos(std::arg(integral));
    y = h * std::abs(integral) * std::sin(std::arg(integral));

    std::cout << "x,"<< x << "y," << y << std::endl;
    std::cout << "arg,"<< std::arg(integral) << std::endl;
    std::cout << "integral,"<< integral << std::endl;

    //描画関数
    float xRend = -x + 100,yRend = -y + 100;
    clothoidPt[i] = Point(yRend,xRend);//Clothoid x,y rendering
    circle(plotImage, clothoidPt[i], 5, cv::Scalar(200,100,0), -1, CV_AA);

  }
  namedWindow("drawing", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
  imshow("drawing", plotImage);
  waitKey(0);
}
