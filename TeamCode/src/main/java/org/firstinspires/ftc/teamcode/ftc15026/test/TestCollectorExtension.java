package org.firstinspires.ftc.teamcode.ftc15026.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(group = "Test")
public class TestCollectorExtension extends OpMode {
    private static final double MAX_ACCELERATION = 1d; //motor power / s^2
    private static final double MAX_JERK = 1000d; //motor power / s^3

    double lastPower = 0d;
    double lastAcceleration = 0d;

    DcMotor collectorExtension;

    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        collectorExtension = hardwareMap.get(DcMotor.class, "feederExtension");
        //feederExtension.setDirection(DcMotorSimple.Direction.REVERSE);

        collectorExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        collectorExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        double dt = runtime.seconds();
        runtime.reset();
        double power = gamepad1.right_stick_x;
        double acceleration = (power - lastPower) / dt;
        double jerk = (acceleration - lastAcceleration) / dt;
        /*if(jerk > MAX_JERK) {
            jerk = MAX_JERK;
            acceleration = jerk * dt + lastAcceleration;
        } else if(jerk < -MAX_JERK) {
            jerk = -MAX_JERK;
            acceleration = -jerk * dt + lastAcceleration;
        }*/

        if(acceleration > MAX_ACCELERATION) {
            acceleration = MAX_ACCELERATION;
            power = MAX_ACCELERATION * dt + lastPower;
        } else if(acceleration < -MAX_ACCELERATION) {
            acceleration = -MAX_ACCELERATION;
            power = -MAX_ACCELERATION * dt + lastPower;
        } else {
            //power = acceleration * dt + lastPower;
        }

        //power = acceleration * dt + lastPower;

        double kS = 0.15d;
        power = Math.abs(power) < kS ? Math.signum(acceleration) == Math.signum(power) ? Math.signum(power) * kS : 0d : power;
        collectorExtension.setPower(Range.clip(power, -0.5d, 0.5d));
        lastPower = collectorExtension.getPower();
        lastAcceleration = acceleration;

        telemetry.addLine("Collector Extension Power: " + collectorExtension.getPower());
        telemetry.addLine("Collector Ticks: " + collectorExtension.getCurrentPosition());
        telemetry.addLine("Acceleration: " + acceleration);
        telemetry.addLine("Jerk: " + jerk);
    }
}
