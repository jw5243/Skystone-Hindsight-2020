package org.firstinspires.ftc.teamcode.ftc15026.subsystems;

import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;

public class Gearbox {
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
        for(RevMotor motor : getMotors()) {
            motor.setPower(power);
        }
    }

    public double getPosition() {
        return getMotors()[0].getPosition();
    }
}
