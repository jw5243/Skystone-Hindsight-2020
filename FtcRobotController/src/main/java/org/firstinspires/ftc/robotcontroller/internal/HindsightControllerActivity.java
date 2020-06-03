package org.firstinspires.ftc.robotcontroller.internal;

import android.os.Bundle;
import android.util.Log;
import android.view.View;

import com.qualcomm.ftcrobotcontroller.R;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.JavaCameraView;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

public class HindsightControllerActivity extends FtcRobotControllerActivity implements CameraBridgeViewBase.CvCameraViewListener2 {
    private static final String TAG = "Hindsight Activity";
    public static JavaCameraView javaCameraView;
    public static Mat cameraFeed;
    public StoneWrangler stoneWrangler;

    private BaseLoaderCallback mLoaderCallBack = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            if(status == LoaderCallbackInterface.SUCCESS) {
                System.loadLibrary("native-lib");
                javaCameraView.setCameraPermissionGranted();
                javaCameraView.enableView();
                stoneWrangler = new StoneWrangler();
                Log.d(TAG, "Successfully loaded Opencv");
            } else {
                Log.d(TAG, "Failed to load Opencv");
            }
        }
    };

    @Override
    public void onCameraViewStarted(int width, int height) {
        cameraFeed = new Mat(width, height, CvType.CV_8UC4);
    }

    @Override
    public void onCameraViewStopped() {

    }

    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        cameraFeed = inputFrame.rgba();
        //VisionLibrary.stoneContour(cameraFeed.getNativeObjAddr());
        //stoneWrangler.analyze(cameraFeed);
        //cameraFeed = stoneWrangler.getVisualization();
        return cameraFeed;
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        javaCameraView = findViewById(R.id.java_camera_view);
        javaCameraView.setVisibility(View.VISIBLE);
        javaCameraView.setCvCameraViewListener(this);
    }

    @Override
    public void onResume() {
        super.onResume();
        if(!OpenCVLoader.initDebug()) {
            boolean success = OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION, this, mLoaderCallBack);
            if(success) {
                Log.d(TAG, "Successfully initialized Opencv");
            } else {
                Log.d(TAG, "Failed to initialize Opencv");
            }
        } else {
            Log.d(TAG, "Successfully initialized Opencv. Using it");
            mLoaderCallBack.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
    }
}
