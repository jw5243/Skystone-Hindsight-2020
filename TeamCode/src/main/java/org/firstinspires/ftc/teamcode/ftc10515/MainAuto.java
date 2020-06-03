package org.firstinspires.ftc.teamcode.ftc10515;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.control.MecanumDriveMPC;
import org.firstinspires.ftc.teamcode.lib.control.MecanumDriveRunnableLQR;
import org.firstinspires.ftc.teamcode.lib.control.MecanumRunnableLQR;
import org.firstinspires.ftc.teamcode.lib.physics.MecanumDriveModel;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;
import org.jblas.DoubleMatrix;

@Autonomous(name = "Main Autonomous 10515")
public class MainAuto extends SkystoneRobot {
    public static DoubleMatrix desiredState = new DoubleMatrix(6, 1, new double[] {
            5, 0, 0, 0, Math.toRadians(0), 0
    });

    @Override
    public void init() {
        super.init();
        setDriveMPC(new MecanumDriveMPC(true));
        setDriveModel(new MecanumDriveModel(100, 0.001d, 12d, 0.315d, 0.315d * (0.1 * 0.1 + 0.032 * 0.032) / 2,
                0.315d * (3 * (0.1 * 0.1 + 0.032 * 0.032) + 0.05 * 0.05) / 12, 0.3d,
                getWheelRadius() * 0.0254d, 13.7d, 0.5d, 12d, 0.187d,
                9.2d, 435 * 2 * Math.PI / 60d, 0.25d, 0.6d,
                getL1() * 0.0254d, getL2() * 0.0254d, getD1() * 0.0254d, getD2() * 0.0254d));
        //getDriveMPC().model = getDriveModel();
        //getDriveMPC().runLQR(desiredState);
        setRunnableLQR(new MecanumRunnableLQR());
        new Thread(getRunnableLQR()).start();
    }

    @Override
    public void loop() {
        super.loop();
        getRunnableLQR().updateMPC();
        //getDrive().drive(getDriveMPC().getOptimalInput((int)((getRunnableLQR().getTimeProfiler().getDeltaTime(TimeUnits.SECONDS, false)
        //        + getRunnableLQR().getPolicyLag()) / getDt()), getRobotStateEstimator().getDriveState(),
        //        getDrive().getLastInput(), isOptimizeInputChange()));
    }
}
