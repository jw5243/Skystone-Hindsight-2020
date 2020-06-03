
package org.firstinspires.ftc.teamcode.ftc10515.subsystems;

import org.firstinspires.ftc.teamcode.ftc10515.Robot;
import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Translation2d;
import org.jblas.DoubleMatrix;

public class Drive implements Subsystem {
    private static final double ACCELERATION_CAP = 2d;
    private static Drive instance;
    private static double lastPower;

    public static Drive getInstance(RevMotor backLeft, RevMotor frontLeft, RevMotor backRight, RevMotor frontRight) {
        if(instance == null) {
            instance = new Drive(backLeft, frontLeft, backRight, frontRight);
        }

        return instance;
    }

    public static Drive getInstance() {
        return instance;
    }

    private RevMotor backLeft;
    private RevMotor frontLeft;
    private RevMotor backRight;
    private RevMotor frontRight;

    public Drive(RevMotor backLeft, RevMotor frontLeft, RevMotor backRight, RevMotor frontRight) {
        setBackLeft(backLeft);
        setFrontLeft(frontLeft);
        setBackRight(backRight);
        setFrontRight(frontRight);
        setLastPower(0d);
    }

    @Override
    public void start() {

    }

    @Override
    public void update(double dt) {
        drive(dt);
    }

    @Override
    public void stop() {
        drive(Pose2d.identity());
    }

    public void drive(final double dt) {
        drive(Robot.getDrivetrainPower(), dt);
    }

    public void drive() {
        drive(Robot.getDrivetrainPower());
    }

    public void drive(double backLeftPower, double frontLeftPower, double backRightPower, double frontRightPower) {
        double backLeftPowerMagnitude   = Math.abs(backLeftPower);
        double frontLeftPowerMagnitude  = Math.abs(frontLeftPower);
        double backRightPowerMagnitude  = Math.abs(backRightPower);
        double frontRightPowerMagnitude = Math.abs(frontRightPower);

        double highestPower = backLeftPowerMagnitude;
        if(frontLeftPowerMagnitude > highestPower) {
            highestPower = frontLeftPowerMagnitude;
        }

        if(backRightPowerMagnitude > highestPower) {
            highestPower = backRightPowerMagnitude;
        }

        if(frontRightPowerMagnitude > highestPower) {
            highestPower = frontRightPowerMagnitude;
        }

        double normalizationFactor = highestPower > 1d ? 1d / highestPower : 1d;
        backLeftPower *= normalizationFactor;
        frontLeftPower *= normalizationFactor;
        backRightPower *= -normalizationFactor;
        frontRightPower *= -normalizationFactor;

        getBackLeft().setPower(backLeftPower);
        getFrontLeft().setPower(frontLeftPower);
        getBackRight().setPower(backRightPower);
        getFrontRight().setPower(frontRightPower);
    }

    public void drive(final Pose2d power, final double dt) {
        double robotPower = power.getTranslation().norm() + Math.abs(power.getRotation().getRadians());
        double x = power.getTranslation().x();
        double y = power.getTranslation().y();
        double turn = power.getRotation().getRadians();

        double acceleration = (robotPower - getLastPower()) / dt;
        if(acceleration > getAccelerationCap()) {
            robotPower = getAccelerationCap() * dt + getLastPower();
        } if(acceleration < -getAccelerationCap()) {
            robotPower = -getAccelerationCap() * dt + getLastPower();
        }

        setLastPower(robotPower);

        double v1 = -x + y - (Robot.getL1() + Robot.getD1()) * turn / Robot.getRobotLength();
        double v2 = -x - y + (Robot.getL1() + Robot.getD2()) * turn / Robot.getRobotLength();
        double v3 = -x - y - (Robot.getL2() + Robot.getD1()) * turn / Robot.getRobotLength();
        double v4 = -x + y + (Robot.getL2() + Robot.getD2()) * turn / Robot.getRobotLength();

        v1 += 0.15d * Math.signum(v1);
        v2 += 0.15d * Math.signum(v2);
        v3 += 0.15d * Math.signum(v3);
        v4 += 0.15d * Math.signum(v4);

        double highestPower = v1;
        if(v2 > highestPower) {
            highestPower = v2;
        }

        if(v3 > highestPower) {
            highestPower = v3;
        }

        if(v4 > highestPower) {
            highestPower = v4;
        }

        double normalizationFactor = highestPower > 1d ? 1d / highestPower : 1d;
        v1 *= normalizationFactor * robotPower;
        v2 *= normalizationFactor * robotPower;
        v3 *= normalizationFactor * robotPower;
        v4 *= normalizationFactor * robotPower;

        getBackLeft().setPower(v3);
        getFrontLeft().setPower(v1);
        getBackRight().setPower(v4);
        getFrontRight().setPower(v2);
    }

    public void drive(final DoubleMatrix input) {
        drive(input.get(0), input.get(1), input.get(2), input.get(3));
    }

    public void drive(final Pose2d power) {
        double backLeftPower   = -power.getTranslation().x() - power.getTranslation().y() - (Robot.getL2() + Robot.getD1()) * power.getRotation().getRadians() / Robot.getRobotLength();
        double frontLeftPower  = -power.getTranslation().x() + power.getTranslation().y() - (Robot.getL1() + Robot.getD1()) * power.getRotation().getRadians() / Robot.getRobotLength();
        double backRightPower  = -power.getTranslation().x() + power.getTranslation().y() + (Robot.getL2() + Robot.getD2()) * power.getRotation().getRadians() / Robot.getRobotLength();
        double frontRightPower = -power.getTranslation().x() - power.getTranslation().y() + (Robot.getL1() + Robot.getD2()) * power.getRotation().getRadians() / Robot.getRobotLength();

        double backLeftPowerMagnitude   = Math.abs(backLeftPower);
        double frontLeftPowerMagnitude  = Math.abs(frontLeftPower);
        double backRightPowerMagnitude  = Math.abs(backRightPower);
        double frontRightPowerMagnitude = Math.abs(frontRightPower);

        double highestPower = backLeftPowerMagnitude;
        if(frontLeftPowerMagnitude > highestPower) {
            highestPower = frontLeftPowerMagnitude;
        }

        if(backRightPowerMagnitude > highestPower) {
            highestPower = backRightPowerMagnitude;
        }

        if(frontRightPowerMagnitude > highestPower) {
            highestPower = frontRightPowerMagnitude;
        }

        double normalizationFactor = highestPower > 1d ? 1d / highestPower : 1d;
        backLeftPower *= normalizationFactor;
        frontLeftPower *= normalizationFactor;
        backRightPower *= normalizationFactor;
        frontRightPower *= normalizationFactor;

        getBackLeft().setPower(backLeftPower);
        getFrontLeft().setPower(frontLeftPower);
        getBackRight().setPower(backRightPower);
        getFrontRight().setPower(frontRightPower);
    }

    public DoubleMatrix getLastInput() {
        return new DoubleMatrix(4, 1, new double[] {
                getBackLeft().getLastPower(),
                getFrontLeft().getLastPower(),
                getBackRight().getLastPower(),
                getFrontRight().getLastPower()
        });
    }

    public RevMotor getBackLeft() {
        return backLeft;
    }

    public void setBackLeft(RevMotor backLeft) {
        this.backLeft = backLeft;
    }

    public RevMotor getFrontLeft() {
        return frontLeft;
    }

    public void setFrontLeft(RevMotor frontLeft) {
        this.frontLeft = frontLeft;
    }

    public RevMotor getBackRight() {
        return backRight;
    }

    public void setBackRight(RevMotor backRight) {
        this.backRight = backRight;
    }

    public RevMotor getFrontRight() {
        return frontRight;
    }

    public void setFrontRight(RevMotor frontRight) {
        this.frontRight = frontRight;
    }

    public static double getLastPower() {
        return lastPower;
    }

    public static void setLastPower(double lastPower) {
        Drive.lastPower = lastPower;
    }

    public static double getAccelerationCap() {
        return ACCELERATION_CAP;
    }
}
