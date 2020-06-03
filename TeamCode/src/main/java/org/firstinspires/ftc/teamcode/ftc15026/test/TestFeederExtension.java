package org.firstinspires.ftc.teamcode.ftc15026.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(group = "Test")
public class TestFeederExtension extends OpMode {
    DcMotor feederExtension1;
    DcMotor feederExtension2;

    @Override
    public void init() {
        feederExtension1 = hardwareMap.get(DcMotor.class, "feederExtension1");
        feederExtension2 = hardwareMap.get(DcMotor.class, "feederExtension2");

        feederExtension1.setDirection(DcMotorSimple.Direction.REVERSE);
        feederExtension2.setDirection(DcMotorSimple.Direction.REVERSE);

        feederExtension1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        feederExtension2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        feederExtension1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        feederExtension2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        feederExtension1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        feederExtension2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        feederExtension1.setPower(-gamepad1.left_stick_y);
        feederExtension2.setPower(-gamepad1.left_stick_y);
        telemetry.addLine("Power: " + feederExtension1.getPower());
        telemetry.addLine("Left Encoder Position: " + feederExtension1.getCurrentPosition());
        telemetry.addLine("Right Encoder Position: " + feederExtension2.getCurrentPosition());
    }
}
