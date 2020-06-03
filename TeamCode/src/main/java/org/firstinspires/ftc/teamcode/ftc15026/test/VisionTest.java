package org.firstinspires.ftc.teamcode.ftc15026.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcontroller.internal.HindsightControllerActivity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.ftc15026.control.EnhancedGamepad;
import org.firstinspires.ftc.teamcode.lib.vision.SkystoneNavigation;
import org.firstinspires.ftc.teamcode.lib.vision.skyloc.StoneWrangler;

@TeleOp(group = "Test")
public class VisionTest extends OpMode {
    volatile SkystoneNavigation skystoneNavigation;
    StoneWrangler stoneWrangler;

    int cameraID;

    ElapsedTime runtime = new ElapsedTime();
    EnhancedGamepad enhancedGamepad;

    @Override
    public void init() {
        skystoneNavigation = new SkystoneNavigation();
        stoneWrangler = new StoneWrangler();
        cameraID = 0;
        enhancedGamepad = new EnhancedGamepad(gamepad1);
        new Thread(() -> {
            skystoneNavigation.init(hardwareMap, telemetry, VuforiaLocalizer.CameraDirection.BACK, true);
            while(true) {
                skystoneNavigation.loop();
                try {
                    Thread.sleep(20);
                } catch(InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }).start();
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        enhancedGamepad.update();
        /*if(HindsightControllerActivity.cameraFeed != null) {
            telemetry.addLine(HindsightControllerActivity.cameraFeed.size().toString());
            telemetry.update();
            stoneWrangler.analyze(HindsightControllerActivity.cameraFeed);
            stoneWrangler.reportStonePosition(telemetry);
        }

        if(enhancedGamepad.isaJustPressed()) {
        //if(runtime.seconds() > 3d) {
            swapCamera();
            telemetry.addLine(HindsightControllerActivity.cameraFeed.size().toString());
            telemetry.update();
            runtime.reset();
        }

        telemetry.addLine(HindsightControllerActivity.cameraFeed.size().toString());*/
        telemetry.addLine("Skystone Location: " + skystoneNavigation.queueSkystonLocation());
    }

    @Override
    public void stop() {
        skystoneNavigation.stop();
    }

    public void swapCamera() {
        cameraID = cameraID^1;
        HindsightControllerActivity.javaCameraView.disableView();
        HindsightControllerActivity.javaCameraView.setCameraIndex(cameraID);
        HindsightControllerActivity.javaCameraView.enableView();
    }
}
