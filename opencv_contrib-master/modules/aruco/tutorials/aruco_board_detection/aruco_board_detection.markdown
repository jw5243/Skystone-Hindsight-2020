Detection of ArUco Boards {#tutorial_aruco_board_detection}
==============================

An ArUco Board is a set of markers that acts like a single marker in the sense that it provides a
single pose for the camera.

The most popular board is the one with all the markers in the same plane, since it can be easily printed:

![](images/gboriginal.png)

However, boards are not limited to this arrangement and can represent any 2d or 3d layout.

The difference between a Board and a set of independent markers is that the relative pose between
the markers in the Board is known a priori. This allows that the corners of all the markers can be used for
estimating the pose of the camera respect to the whole Board.

When you use a set of independent markers, you can estimate the pose for each marker individually,
since you dont know the relative pose of the markers in the environment.

The main benefits of using Boards are:

- The pose estimation is much more versatile. Only some markers are necessary to perform pose estimation.
Thus, the pose can be calculated even in the presence of occlusions or partial views.
- The obtained pose is usually more accurate since a higher amount of point correspondences (marker
corners) are employed.

The aruco module allows the use of Boards. The main class is the ```cv::aruco::Board``` class which defines the Board layout:

@code{.cpp}
    class  Board {
    public:
        std::vector<std::vector<cv::Point3f> > objPoints;
        cv::Ptr<cv::aruco::Dictionary> dictionary;
        std::vector<int> ids;
    };
@endcode

A object of type ```Board``` has three parameters:
- The ```objPoints``` structure is the list of corner positions in the 3d Board reference system, i.e. its layout.
For each marker, its four corners are stored in the standard order, i.e. in clockwise order and starting
with the top left corner.
- The ```dictionary``` parameter indicates to which marker dictionary the Board markers belong to.
- Finally, the ```ids``` structure indicates the identifiers of each of the markers in ```objPoints``` respect to the specified  ```dictionary```.


Board Detection
-----

A Board detection is similar to the standard marker detection. The only difference is in the pose estimation step.
In fact, to use marker boards, a standard marker detection should be done before estimating the Board pose.

The aruco module provides a specific function, ```estimatePoseBoard()```, to perform pose estimation for boards:

@code{.cpp}
    cv::Mat inputImage;
    // camera parameters are read from somewhere
    cv::Mat cameraMatrix, distCoeffs;
    readCameraParameters(cameraMatrix, distCoeffs);
    // assume we have a function to create the board object
    cv::Ptr<cv::aruco::Board> board = cv::aruco::Board::create();
    ...
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;
    cv::aruco::detectMarkers(inputImage, board.dictionary, markerCorners, markerIds);
    // if at least one marker detected
    if(markerIds.size() > 0) {
        cv::Vec3d rvec, tvec;
        int valid = cv::aruco::estimatePoseBoard(markerCorners, markerIds, board, cameraMatrix, distCoeffs, rvec, tvec);
    }
@endcode

The parameters of estimatePoseBoard are:

- ```markerCorners``` and ```markerIds```: structures of detected markers from ```detectMarkers()``` function.
- ```board```: the ```Board``` object that defines the board layout and its ids
- ```cameraMatrix``` and ```distCoeffs```: camera calibration parameters necessary for pose estimation.
- ```rvec``` and ```tvec```: estimated pose of the Board. If not empty then treated as initial guess.
- The function returns the total number of markers employed for estimating the board pose. Note that not all the
 markers provided in ```markerCorners``` and ```markerIds``` should be used, since only the markers whose ids are
listed in the ```Board::ids``` structure are considered.

The ```drawAxis()``` function can be used to check the obtained pose. For instance:

![Board with axis](images/gbmarkersaxis.png)

And this is another example with the board partially occluded:

![Board with occlusions](images/gbocclusion.png)

As it can be observed, although some markers have not been detected, the Board pose can still be estimated from the rest of markers.

Grid Board
-----

Creating the ```Board``` object requires specifying the corner positions for each marker in the environment.
However, in many cases, the board will be just a set of markers in the same plane and in a grid layout,
so it can be easily printed and used.

Fortunately, the aruco module provides the basic functionality to create and print these types of markers
easily.

The ```GridBoard``` class is a specialized class that inherits from the ```Board``` class and which represents a Board
with all the markers in the same plane and in a grid layout, as in the following image:

![Image with aruco board](images/gboriginal.png)

Concretely, the coordinate system in a Grid Board is positioned in the board plane, centered in the bottom left
corner of the board and with the Z pointing out, like in the following image (X:red, Y:green, Z:blue):

![Board with axis](images/gbaxis.png)

A ```GridBoard``` object can be defined using the following parameters:

- Number of markers in the X direction.
- Number of markers in the Y direction.
- Length of the marker side.
- Length of the marker separation.
- The dictionary of the markers.
- Ids of all the markers (X*Y markers).

This object can be easily created from these parameters using the ```cv::aruco::GridBoard::create()``` static function:

@code{.cpp}
    cv::aruco::GridBoard board = cv::aruco::GridBoard::create(5, 7, 0.04, 0.01, dictionary);
@endcode

- The first and second parameters are the number of markers in the X and Y direction respectively.
- The third and fourth parameters are the marker length and the marker separation respectively. They can be provided
in any unit, having in mind that the estimated pose for this board will be measured in the same units (in general, meters are used).
- Finally, the dictionary of the markers is provided.

So, this board will be composed by 5x7=35 markers. The ids of each of the markers are assigned, by default, in ascending
order starting on 0, so they will be 0, 1, 2, ..., 34. This can be easily customized by accessing to the ids vector
through ```board.ids```, like in the ```Board``` parent class.

After creating a Grid Board, we probably want to print it and use it. A function to generate the image
of a ```GridBoard``` is provided in ```cv::aruco::GridBoard::draw()```. For example:

@code{.cpp}
    cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(5, 7, 0.04, 0.01, dictionary);
    cv::Mat boardImage;
    board->draw( cv::Size(600, 500), boardImage, 10, 1 );
@endcode

- The first parameter is the size of the output image in pixels. In this case 600x500 pixels. If this is not proportional
to the board dimensions, it will be centered on the image.
- ```boardImage```: the output image with the board.
- The third parameter is the (optional) margin in pixels, so none of the markers are touching the image border.
In this case the margin is 10.
- Finally, the size of the marker border, similarly to ```drawMarker()``` function. The default value is 1.

The output image will be something like this:

![](images/board.jpg)

A full working example of board creation is included in the ```create_board.cpp``` inside the module samples folder.

Note: The samples now take input via commandline via the [OpenCV Commandline Parser](http://docs.opencv.org/trunk/d0/d2e/classcv_1_1CommandLineParser.html#gsc.tab=0). For this file the example parameters will look like
@code{.cpp}
    "_output path_/aboard.png" -w=5 -h=7 -l=100 -s=10 -d=10
@endcode

Finally, a full example of board detection:

@code{.cpp}
    cv::VideoCapture inputVideo;
    inputVideo.open(0);

    cv::Mat cameraMatrix, distCoeffs;
    // camera parameters are read from somewhere
    readCameraParameters(cameraMatrix, distCoeffs);

    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(5, 7, 0.04, 0.01, dictionary);

    while (inputVideo.grab()) {
        cv::Mat image, imageCopy;
        inputVideo.retrieve(image);
        image.copyTo(imageCopy);

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f> > corners;
        cv::aruco::detectMarkers(image, dictionary, corners, ids);

        // if at least one marker detected
        if (ids.size() > 0) {
            cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);

            cv::Vec3d rvec, tvec;
            int valid = estimatePoseBoard(corners, ids, board, cameraMatrix, distCoeffs, rvec, tvec);

            // if at least one board marker detected
            if(valid > 0)
                cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvec, tvec, 0.1);
        }

        cv::imshow("out", imageCopy);
        char key = (char) cv::waitKey(waitTime);
        if (key == 27)
            break;
    }
@endcode

Sample video:

@htmlonly
<iframe width="420" height="315" src="https://www.youtube.com/embed/Q1HlJEjW_j0" frameborder="0" allowfullscreen></iframe>
@endhtmlonly

A full working example is included in the ```detect_board.cpp``` inside the module samples folder.

Note: The samples now take input via commandline via the [OpenCV Commandline Parser](http://docs.opencv.org/trunk/d0/d2e/classcv_1_1CommandLineParser.html#gsc.tab=0). For this file the example parameters will look like
@code{.cpp}
    -c="_path_"/calib.txt" "_path_/aboard.png" -w=5 -h=7 -l=100 -s=10 -d=10
@endcode



Refine marker detection
-----

ArUco boards can also be used to improve the detection of markers. If we have detected a subset of the markers
that belongs to the board, we can use these markers and the board layout information to try to find the
markers that have not been previously detected.

This can be done using the ```refineDetectedMarkers()``` function, which should be called
after calling ```detectMarkers()```.

The main parameters of this function are the original image where markers were detected, the Board object,
the detected marker corners, the detected marker ids and the rejected marker corners.

The rejected corners can be obtained from the ```detectMarkers()``` function and are also known as marker
candidates. This candidates are square shapes that have been found in the original image but have failed
to pass the identification step (i.e. their inner codification presents too many errors) and thus they
have not been recognized as markers.

However, these candidates are sometimes actual markers that have not been correctly identified due to high
noise in the image, very low resolution or other related problems that affect to the binary code extraction.
The ```refineDetectedMarkers()``` function finds correspondences between these candidates and the missing
markers of the board. This search is based on two parameters:

- Distance between the candidate and the projection of the missing marker. To obtain these projections,
it is necessary to have detected at least one marker of the board. The projections are obtained using the
camera parameters (camera matrix and distortion coefficients) if they are provided. If not, the projections
are obtained from local homography and only planar board are allowed (i.e. the Z coordinate of all the
marker corners should be the same). The ```minRepDistance``` parameter in ```refineDetectedMarkers()```
determines the minimum euclidean distance between the candidate corners and the projected marker corners
(default value 10).

- Binary codification. If a candidate surpasses the minimum distance condition, its internal bits
are analyzed again to determine if it is actually the projected marker or not. However, in this case,
the condition is not so strong and the number of allowed erroneous bits can be higher. This is indicated
in the ```errorCorrectionRate``` parameter (default value 3.0). If a negative value is provided, the
internal bits are not analyzed at all and only the corner distances are evaluated.

This is an example of using the  ```refineDetectedMarkers()``` function:

@code{.cpp}
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(5, 7, 0.04, 0.01, dictionary);
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::aruco::detectMarkers(inputImage, dictionary, markerCorners, markerIds, cv::aruco::DetectorParameters(), rejectedCandidates);

    cv::aruco::refineDetectedMarkersinputImage, board, markerCorners, markerIds, rejectedCandidates);
    // After calling this function, if any new marker has been detected it will be removed from rejectedCandidates and included
    // at the end of markerCorners and markerIds
@endcode

It must also be noted that, in some cases, if the number of detected markers in the first place is too low (for instance only
1 or 2 markers), the projections of the missing markers can be of bad quality, producing erroneous correspondences.

See module samples for a more detailed implementation.
