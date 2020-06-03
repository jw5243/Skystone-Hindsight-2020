package org.firstinspires.ftc.teamcode.ftc15026.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(group = "Test")
public class TestSpindle extends OpMode {
    DcMotor spindle;

    @Override
    public void init() {
        spindle = hardwareMap.get(DcMotor.class, "spindle");
    }

    @Override
    public void loop() {
        spindle.setPower(gamepad1.right_stick_x);

        telemetry.addLine("Spindle Power: " + spindle.getPower());
    }
}
