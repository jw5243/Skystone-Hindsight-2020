package org.firstinspires.ftc.teamcode.ftc10515.subsystems;

import org.firstinspires.ftc.teamcode.ftc10515.statemachines.FlywheelStateMachine;
import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;

public class FlywheelCollector implements Subsystem {
    private static FlywheelCollector instance;

    public static FlywheelCollector getInstance(RevMotor leftFlywheels, RevMotor rightFlywheels) {
        if(instance == null) {
            instance = new FlywheelCollector(leftFlywheels, rightFlywheels);
        }

        return instance;
    }

    private RevMotor leftFlywheels;
    private RevMotor rightFlywheels;

    public FlywheelCollector(RevMotor leftFlywheels, RevMotor rightFlywheels) {
        setLeftFlywheels(leftFlywheels);
        setRightFlywheels(rightFlywheels);
    }

    @Override
    public void start() {
        getLeftFlywheels().setPower(-FlywheelStateMachine.State.IDLE.getPower());
        getRightFlywheels().setPower(FlywheelStateMachine.State.IDLE.getPower());
    }

    @Override
    public void update(double dt) {
        getLeftFlywheels().setPower(-FlywheelStateMachine.getState().getPower());
        getRightFlywheels().setPower(FlywheelStateMachine.getState().getPower());
    }

    @Override
    public void stop() {
        getLeftFlywheels().setPower(-FlywheelStateMachine.State.IDLE.getPower());
        getRightFlywheels().setPower(FlywheelStateMachine.State.IDLE.getPower());
    }

    public RevMotor getLeftFlywheels() {
        return leftFlywheels;
    }

    public void setLeftFlywheels(RevMotor leftFlywheels) {
        this.leftFlywheels = leftFlywheels;
    }

    public RevMotor getRightFlywheels() {
        return rightFlywheels;
    }

    public void setRightFlywheels(RevMotor rightFlywheels) {
        this.rightFlywheels = rightFlywheels;
    }
}
