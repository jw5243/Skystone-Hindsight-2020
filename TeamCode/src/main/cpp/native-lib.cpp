#include "opencv2/core.hpp"
//#include "opencv_contrib-master/modules/xfeatures2d/src/sift.cpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <jni.h>
#include "VisionUtil.h"

using namespace cv;
using namespace std;

const Scalar SKYSTONE_HSV_LOWER = Scalar(25, 0, 0);
const Scalar SKYSTONE_HSV_UPPER = Scalar(255, 200, 100);

const Scalar STONE_HSV_LOWER = Scalar(80, 120, 150);
const Scalar STONE_HSV_UPPER = Scalar(120, 255, 255);

const double PI = 3.141592653589793238463;

//const Scalar STONE_HSV_LOWER = Scalar(80, 190, 0);
//const Scalar STONE_HSV_UPPER = Scalar(120, 255, 255);

extern "C"
JNIEXPORT void JNICALL
Java_org_firstinspires_ftc_robotcontroller_internal_VisionLibrary_filterSkystone(JNIEnv *env,
                                                                               jclass clazz,
                                                                               jlong addrRgba) {
    Mat& img = *(Mat*)(addrRgba);
    GaussianBlur(img, img, Size(9, 9), BORDER_DEFAULT);
    Mat hsvFrame = Mat(img.rows, img.cols, CV_8U, Scalar(3));
    cvtColor(img, hsvFrame, COLOR_BGR2HSV, 3);
    inRange(hsvFrame, SKYSTONE_HSV_LOWER, SKYSTONE_HSV_UPPER, img);
    hsvFrame.release();
}

extern "C"
JNIEXPORT void JNICALL
Java_org_firstinspires_ftc_robotcontroller_internal_VisionLibrary_filterStone(JNIEnv *env,
                                                                              jclass clazz,
                                                                              jlong addrRgba) {
    Mat& img = *(Mat*)(addrRgba);
    GaussianBlur(img, img, Size(9, 9), BORDER_DEFAULT);
    dilate(img, img, 0, Point(-1, -1), 2, BORDER_DEFAULT, 1);
    Mat hsvFrame = Mat(img.rows, img.cols, CV_8U, Scalar(3));
    cvtColor(img, hsvFrame, COLOR_BGR2HSV, 3);
    inRange(hsvFrame, STONE_HSV_LOWER, STONE_HSV_UPPER, img);
    threshold(img, img, 100, 255, THRESH_BINARY);
    hsvFrame.release();
}

extern "C"
JNIEXPORT jobjectArray JNICALL
Java_org_firstinspires_ftc_robotcontroller_internal_VisionLibrary_skystoneContour(JNIEnv *env,
                                                                                  jclass clazz,
                                                                                  jlong addrRgba) {
    jclass       point3Class     = env->FindClass("org/opencv/core/Point3");
    jobjectArray skystoneCorners = env->NewObjectArray(4, point3Class, nullptr);

    Mat& img = *(Mat*)(addrRgba);
    GaussianBlur(img, img, Size(9, 9), 0, 0, BORDER_DEFAULT);

    Mat hsvFrame = Mat(img.rows, img.cols, CV_8U, Scalar(3));
    cvtColor(img, hsvFrame, COLOR_BGR2HSV, 3);
    inRange(hsvFrame, SKYSTONE_HSV_LOWER, SKYSTONE_HSV_UPPER, hsvFrame);
    threshold(hsvFrame, hsvFrame, 100, 255, THRESH_BINARY_INV);
    //bitwise_and(img, Scalar(1.0f, 1.0f, 1.0f), img, hsvFrame);
    hsvFrame.release();

    vector<vector<Point>> rectangles;
    getRectangles(img, rectangles);
    sort(rectangles.begin(), rectangles.end(), compareContourAreas);
    drawRectangles(rectangles, img);

    return skystoneCorners;
}

extern "C"
JNIEXPORT jobjectArray JNICALL
Java_org_firstinspires_ftc_robotcontroller_internal_VisionLibrary_stoneContour(JNIEnv *env,
                                                                               jclass clazz,
                                                                               jlong addrRgba) {
    jclass       point3Class     = env->FindClass("org/opencv/core/Point3");
    jobjectArray skystoneCorners = env->NewObjectArray(4, point3Class, nullptr);

    Mat& img = *(Mat*)(addrRgba);
    GaussianBlur(img, img, Size(9, 9), 0, 0, BORDER_DEFAULT);

    Mat hsvFrame = Mat(img.rows, img.cols, CV_8U, Scalar(3));
    cvtColor(img, hsvFrame, COLOR_BGR2HSV, 3);
    inRange(hsvFrame, STONE_HSV_LOWER, STONE_HSV_UPPER, hsvFrame);
    threshold(hsvFrame, hsvFrame, 100, 255, THRESH_BINARY_INV);
    bitwise_and(img, Scalar(1.0f, 1.0f, 1.0f), img, hsvFrame);
    hsvFrame.release();

    vector<vector<Point>> rectangles;
    getRectangles(img, rectangles);
    sort(rectangles.begin(), rectangles.end(), compareContourAreas);
    drawRectangles(rectangles, img);

    /*Mat dst_norm;
    cornerHarris(img, img, 2, 3, 0.04, BORDER_DEFAULT);
    normalize(img, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat());
    convertScaleAbs(dst_norm, img);
    for(int i = 0; i < img.rows; i++) {
        for(int j = 0; j < img.cols; j++) {
            if((int)(dst_norm.at<float>(i, j)) > 200) {
                circle(img, Point(j, i), 5, Scalar(0), 2, 8, 0);
            }
        }
    }

    dst_norm.release();*/

    return skystoneCorners;
}

extern "C"
JNIEXPORT void JNICALL
Java_org_firstinspires_ftc_robotcontroller_internal_VisionLibrary_stoneDetector(JNIEnv *env,
                                                                                jclass clazz,
                                                                                jlong addrRgba) {
    Mat& img = *(Mat*)(addrRgba);

    const vector<Point2f> calibrationFramePoints = {Point2f(280, 124), Point2f(550, 124), Point2f(180, 260), Point2f(710, 260)};
    const vector<Point2f> calibrationWorldPoints = {Point2f(0, 9.5 + 11), Point2f(8.5, 9.5 + 11), Point2f(0, 9.5), Point2f(8.5, 9.5)};
    const double pixelSize = 0.1;
    const double areaXDimension = 45;
    const double areaYDimension = 45;
    const double houghLinesRhoStep = 1;
    const double houghLinesThetaStep = PI / 180;
    const Scalar redScalar = Scalar(0, 0, 255);
    const Scalar greenScalar = Scalar(0, 255, 0);
    const Scalar blueScalar = Scalar(255, 0, 0);
    const double rhoThreshold = 40;
    const double thetaThreshld = PI * 15 / 180;

    Mat homography = findHomography(calibrationFramePoints, calibrationWorldPoints);

    GaussianBlur(img, img, Size(9, 9), 0, 0, BORDER_DEFAULT);
}

extern "C"
JNIEXPORT void JNICALL
Java_org_firstinspires_ftc_robotcontroller_internal_VisionLibrary_skystoneDetector(JNIEnv *env,
                                                                                   jclass clazz,
                                                                                   jlong addrRgba) {
    Mat& img = *(Mat*)(addrRgba);
    GaussianBlur(img, img, Size(9, 9), 0, 0, BORDER_DEFAULT);

    Mat hsvFrame = Mat(img.rows, img.cols, CV_8U, Scalar(3));
    cvtColor(img, hsvFrame, COLOR_BGR2HSV, 3);
    inRange(hsvFrame, STONE_HSV_LOWER, STONE_HSV_UPPER, hsvFrame);
    threshold(hsvFrame, hsvFrame, 100, 255, THRESH_BINARY_INV);
    bitwise_and(img, Scalar(1.0f, 1.0f, 1.0f), img, hsvFrame);
    hsvFrame.release();

    Mat gray0(img.size(), CV_8U);
    Mat gray;
    vector<vector<Point>> rectangles;
    vector<vector<Point>> contours;
    for(int c = 0; c < 3; c++) {
        int ch[] = {c, 0};
        mixChannels(&img, 1, &gray0, 1, ch, 1);
        const int threshold_level = 2;
        for(int l = 0; l < threshold_level; l++) {
            if(l == 0) {
                Canny(gray0, gray, 20, 30, 3, false);
                dilate(gray, gray, Mat(), Point(-1, -1));
            } else {
                gray = gray0 >= (l + 1) * 255 / threshold_level;
            }

            findContours(gray, contours, RETR_CCOMP, CHAIN_APPROX_SIMPLE);
            vector<Point> approx;
            for(size_t i = 0; i < contours.size(); i++) {
                approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true) * 0.01, false);
                if(approx.size() == 4 && fabs(contourArea(Mat(approx))) > 1000 && isContourConvex(Mat(approx))) {
                    double maxCosine = 0;
                    for(int j = 2; j < 5; j++) {
                        double cosine = fabs(angle(approx[j % 4], approx[j - 2], approx[j - 1]));
                        maxCosine = MAX(maxCosine, cosine);
                    }

                    if(maxCosine < 0.3) {
                        rectangles.push_back(approx);
                    }
                }
            }
        }
    }

    gray.release();
    gray0.release();

    sort(rectangles.begin(), rectangles.end(), compareContourAreas);
    drawRectangles(rectangles, img);
}