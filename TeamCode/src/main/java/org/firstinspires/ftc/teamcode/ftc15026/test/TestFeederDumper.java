package org.firstinspires.ftc.teamcode.ftc15026.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ftc15026.control.EnhancedGamepad;

@TeleOp(group = "Test")
public class TestFeederDumper extends OpMode {
    Servo feederDumper;

    EnhancedGamepad enhancedGamepad1;

    @Override
    public void init() {
        feederDumper = hardwareMap.get(Servo.class, "feederDumper");
        enhancedGamepad1 = new EnhancedGamepad(gamepad1);
    }

    @Override
    public void start() {
        feederDumper.setPosition(0.5d);
    }

    @Override
    public void loop() {
        enhancedGamepad1.update();
        if(enhancedGamepad1.isxJustPressed()) {
            feederDumper.setPosition(feederDumper.getPosition() - 0.05d);
        } else if(enhancedGamepad1.isyJustPressed()) {
            feederDumper.setPosition(feederDumper.getPosition() + 0.05d);
        }

        telemetry.addLine("Feeder Dumper Position: " + feederDumper.getPosition());
    }
}
