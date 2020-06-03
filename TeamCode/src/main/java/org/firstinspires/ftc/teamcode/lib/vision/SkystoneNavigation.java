package org.firstinspires.ftc.teamcode.lib.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

public class SkystoneNavigation {
    private static final boolean PHONE_IS_PORTRAIT = false;

    private static final String VUFORIA_KEY = "AZxwGtv/////AAABmS9S0zwWtEtnoBg8dHWbs5FOMyiWOdxpcgiCdnH7fMYPevFRGn9UkImNkmouzKPLjCTIqajY3ev+uUQfFMaZfogR3DGRwhqwGI2yhZmzSTdmt0R35J++N54XsXIXN58k+ouUh+mRoroocA/DQh4OtuRrBb59xoykl0cSIylX3Nls0klV+ELZhnGKKdM32rtXnZsw1WAApsU8hwO2hnfmJn/F6iYI51o+BndFs4zn+q8h8H6s/zNo61uStnnTHnHDatlQXr44A423g6ahOndB4kqLlF+nZWqy4FfHEZcUCNgiFPerWE2mhG6aBGP3XfLPMg2E0+zh1MClQtpV4athSB3QIS4y/voM9YzHOY5Tl6EM";

    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch; // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Class Members
    private volatile OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private volatile boolean targetVisible = false;
    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private float phoneZRotate = 0;

    private VuforiaLocalizer.CameraDirection cameraChoice;
    private HardwareMap hardwareMap;
    private volatile Telemetry telemetry;

    private VuforiaTrackables targetsSkyStone;
    private List<VuforiaTrackable> allTrackables;

    public volatile Pose2d updatedPosition;

    public void toggleCameraDirection() {
        vuforia.getCamera().close();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        cameraChoice = cameraChoice.ordinal() == VuforiaLocalizer.CameraDirection.FRONT.ordinal() ? BACK : FRONT;

        parameters.cameraDirection = cameraChoice;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry, VuforiaLocalizer.CameraDirection cameraDirection, boolean stream) {
        this.cameraChoice = cameraDirection;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = cameraChoice;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");

        allTrackables = new ArrayList<>();
        allTrackables.addAll(targetsSkyStone);

        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        if(cameraChoice == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if(PHONE_IS_PORTRAIT) {
            phoneXRotate = 90;
        }

        final float CAMERA_FORWARD_DISPLACEMENT = 0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 3f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 5f;

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        for(VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        if(stream) {
            targetsSkyStone.activate();
        }
    }

    public void loop() {
        targetVisible = false;
        for(VuforiaTrackable trackable : allTrackables) {
            if(((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                targetVisible = true;
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if(robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }

                break;
            }
        }

        if(targetVisible) {
            VectorF translation = lastLocation.getTranslation();
            //telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
            //        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            //telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            updatedPosition = new Pose2d(translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, new Rotation2d(rotation.thirdAngle * Math.PI / 180d, false));
        } else {
            //telemetry.addData("Visible Target", "none");
        }
    }

    public Pose2d queueSkystonLocation() {
        return targetVisible ? updatedPosition : null;
    }

    public void stop() {
        targetsSkyStone.deactivate();
    }
}
