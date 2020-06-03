#include <opencv2/core.hpp>

using namespace cv;

const double PI = 3.14159;
//Define frame space calibration points
const double CALX1F = 629;
const double CALY1F = 413;
const double CALX2F = 935;
const double CALY2F = 412;
const double CALX3F = 1067;
const double CALY3F = 228;
const double CALX4F = 635;
const double CALY4F = 228;

//Define world space calibration points
const double CALX1W = 0;
const double CALY1W = 11 * 3;
const double CALX2W = 8.5;
const double CALY2W = 11 * 3;
const double CALX3W = 8.5;
const double CALY3W = 11 * 2;
const double CALX4W = 0;
const double CALY4W = 11 * 2;

//world size of bird's eye view pixel (e.g. .1 inch -> 1 px == .1)
const double PIXEL_SIZE = 0.1;

//Surveyed area's dimensions in world space
const double AREA_X_DIMENSION = 45;
const double AREA_Y_DIMENSION = 45;

const Size GAUSSIAN_DENOISE_K = *new Size(3, 3);

const Scalar STONE_HSV_LOWER = *new Scalar(.09 * 179, .7 * 255, 0.1* 255);
const Scalar STONE_HSV_UPPER = *new Scalar(.14 * 179, 1. * 255, 1. * 255);

const double HOUGH_LINES_RHO_STEP = 1;
const double HOUGH_LINES_THETA_STEP = PI / 180;

const int HOUGH_LINES_THRESHOLD = 10;

const Scalar RED_SCALAR = *new Scalar(0, 0, 255);
const Scalar GREEN_SCALAR = *new Scalar(0, 255, 0);
const Scalar BLUE_SCALAR = *new Scalar(255, 0, 0);

const double CLOSE_ENOUGH_THETA = PI * 15 / 180;
const double CLOSE_ENOUGH_RHO = 40;

