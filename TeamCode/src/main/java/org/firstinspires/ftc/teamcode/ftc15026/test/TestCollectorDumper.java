package org.firstinspires.ftc.teamcode.ftc15026.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ftc15026.control.EnhancedGamepad;

@TeleOp(group = "Test")
public class TestCollectorDumper extends OpMode {
    Servo collectorDumper;

    EnhancedGamepad enhancedGamepad1;

    @Override
    public void init() {
        collectorDumper = hardwareMap.get(Servo.class, "collectorDumper");
        enhancedGamepad1 = new EnhancedGamepad(gamepad1);
    }

    @Override
    public void start() {
        collectorDumper.setPosition(0.5d);
    }

    @Override
    public void loop() {
        enhancedGamepad1.update();
        if(enhancedGamepad1.isxJustPressed()) {
            collectorDumper.setPosition(collectorDumper.getPosition() - 0.05d);
        } else if(enhancedGamepad1.isyJustPressed()) {
            collectorDumper.setPosition(collectorDumper.getPosition() + 0.05d);
        }

        telemetry.addLine("Collector Dumper Position: " + collectorDumper.getPosition());
    }
}
