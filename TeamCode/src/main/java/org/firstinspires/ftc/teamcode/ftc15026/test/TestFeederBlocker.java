package org.firstinspires.ftc.teamcode.ftc15026.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ftc15026.control.EnhancedGamepad;

@TeleOp(group = "Test")
public class TestFeederBlocker extends OpMode {
    Servo feederBlocker;
    EnhancedGamepad enhancedGamepad;

    @Override
    public void init() {
        feederBlocker = hardwareMap.get(Servo.class, "feederStopper");
        enhancedGamepad = new EnhancedGamepad(gamepad1);
        feederBlocker.setPosition(0.5d);
    }

    @Override
    public void loop() {
        enhancedGamepad.update();
        if(enhancedGamepad.isDpadUpJustPressed()) {
            feederBlocker.setPosition(feederBlocker.getPosition() + 0.05d);
        } else if(enhancedGamepad.isDpadDownJustPressed()) {
            feederBlocker.setPosition(feederBlocker.getPosition() - 0.05d);
        }

        telemetry.addLine("Feeder Blocker Position: " + feederBlocker.getPosition());
    }
}
