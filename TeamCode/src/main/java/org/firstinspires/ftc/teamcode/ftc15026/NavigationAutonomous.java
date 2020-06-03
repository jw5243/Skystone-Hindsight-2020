package org.firstinspires.ftc.teamcode.ftc15026;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;

@Autonomous
public class NavigationAutonomous extends SkystoneRobot {
    ElapsedTime runtime = new ElapsedTime();
    int step = 0;

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        if(step == 0) {
            //setDrivetrainPower(new Pose2d(0d, 0.5d, new Rotation2d(0d, true)));
            getDrive().drive(0.3, 0.3, 0.3, 0.3);
            while(runtime.milliseconds() < 1000);
            step++;
        } else {
            getDrive().drive(0, 0, 0, 0);
        }
    }
}
