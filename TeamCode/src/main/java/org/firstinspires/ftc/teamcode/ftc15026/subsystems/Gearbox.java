package org.firstinspires.ftc.teamcode.ftc15026.subsystems;

import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;

public class Gearbox {
    private static final double kP = 0.1d;
    private RevMotor[] motors;

    public Gearbox(RevMotor motor) {
        this(new RevMotor[] {motor});
    }

    public Gearbox(RevMotor motor1, RevMotor motor2) {
        this(new RevMotor[] {motor1, motor2});
    }

    public Gearbox(RevMotor[] motors) {
        setMotors(motors);
    }

    public RevMotor[] getMotors() {
        return motors;
    }

    public void setMotors(RevMotor[] motors) {
        this.motors = motors;
    }

    public void setPower(double power) {
        if(getMotors() == null) {
            return;
        }

        getMotors()[0].setPower(power);

        //Let the first motor in the list be the master and have the other motors follow.
        //This will only run if there is more than one motor in the list.
        for(int i = 1; i < getMotors().length; i++) {
            double error = getMotors()[0].getPosition() - getMotors()[i].getPosition();
            getMotors()[i].setPower(power); //+ kP * error);
        }
    }

    public double getPower() {
        return getMotors()[0].getLastPower();
    }

    public double getPosition() {
        return getMotors()[0].getPosition();
    }

    public double getVelocity() {
        return getMotors()[0].getVelocity();
    }
}
