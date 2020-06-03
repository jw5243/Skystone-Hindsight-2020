package org.firstinspires.ftc.teamcode.ftc15026.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.ftc15026.control.EnhancedGamepad;

@TeleOp(group = "Test")
public class TestStaticFrictionDrive extends OpMode {
    DcMotor backLeft, frontLeft, backRight, frontRight;
    EnhancedGamepad enhancedGamepad;
    double power;

    @Override
    public void init() {
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        enhancedGamepad = new EnhancedGamepad(gamepad1);
        power = 0d;
    }

    @Override
    public void loop() {
        enhancedGamepad.update();
        if(enhancedGamepad.isDpadUpJustPressed()) {
            power += 0.01d;
        } else if(enhancedGamepad.isDpadDownJustPressed()) {
            power -= 0.01d;
        }

        backLeft.setPower(power);
        frontLeft.setPower(power);
        backRight.setPower(power);
        frontRight.setPower(power);

        telemetry.addLine("Power: " + power);
    }
}
