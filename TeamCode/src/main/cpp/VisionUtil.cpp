#include "VisionUtil.h"

double angle(Point& pt1, Point& pt2, Point& pt0) {
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1E-10);
}

Mat drawRectangles(vector<vector<Point>>& rectangles, Mat& image) {
    if(rectangles.empty()) {
        return image;
    }

    for(int i = 0; i < 1/*rectangles.size()*/; i++) {
        //draw contour
        drawContours(image, rectangles, i, Scalar(255, 0, 0), 1, 8, vector<Vec4i>(), 0, Point());

        //draw bounding rect
        Rect rect = boundingRect(Mat(rectangles[i]));
        rectangle(image, rect.tl(), rect.br(), Scalar(0, 255, 0), 2, 8, 0);

        //draw rotated rect
        RotatedRect minRect = minAreaRect(Mat(rectangles[i]));
        Point2f rect_points[4];
        minRect.points(rect_points);
        for(int j = 0; j < 4; j++) {
            line(image, rect_points[j], rect_points[(j + 1) % 4], Scalar(0, 0, 255), 1, 8);
        }
    }

    return image;
}

bool compareContourAreas(vector<Point>& contour1, vector<Point>& contour2) {
    return fabs(contourArea(Mat(contour1))) > fabs(contourArea(Mat(contour2)));
}

void getRectangles(Mat& img, vector<vector<Point>>& rectangles) {
    Mat gray0(img.size(), CV_8U);
    Mat gray;
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

            findContours(gray, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);
            vector<Point> approx;
            for(size_t i = 0; i < contours.size(); i++) {
                approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true) * 0.01, false);
                /*if(approx.size() == 4 && fabs(contourArea(Mat(approx))) > 1000 && isContourConvex(Mat(approx))) {
                    double maxCosine = 0;
                    for(int j = 2; j < 5; j++) {
                        double cosine = fabs(angle(approx[j % 4], approx[j - 2], approx[j - 1]));
                        maxCosine = MAX(maxCosine, cosine);
                    }

                    if(maxCosine < 1) {
                        rectangles.push_back(approx);
                    }
                }*/

                rectangles.push_back(approx);
            }
        }
    }

    gray.release();
    gray0.release();
}