#ifndef HINDSIGHT_2020_WINTER_BREAK_VISIONUTIL_H
#define HINDSIGHT_2020_WINTER_BREAK_VISIONUTIL_H

#include "opencv2/core.hpp"
#include <opencv2/imgproc.hpp>

using namespace cv;
using namespace std;

double angle(Point& pt1, Point& pt2, Point& pt0);
Mat drawRectangles(vector<vector<Point>>& rectangles, Mat& image);
bool compareContourAreas(vector<Point>& contour1, vector<Point>& contour2);
void getRectangles(Mat& img, vector<vector<Point>>& rectangles);

#endif //HINDSIGHT_2020_WINTER_BREAK_VISIONUTIL_H
