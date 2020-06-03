package org.firstinspires.ftc.teamcode.ftc10515.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Translation2d;
import org.jblas.DoubleMatrix;

import static org.firstinspires.ftc.teamcode.ftc15026.Robot.getD1;
import static org.firstinspires.ftc.teamcode.ftc15026.Robot.getD2;
import static org.firstinspires.ftc.teamcode.ftc15026.Robot.getL1;
import static org.firstinspires.ftc.teamcode.ftc15026.Robot.getL2;
import static org.firstinspires.ftc.teamcode.ftc15026.Robot.getWheelRadius;

public class RobotStateEstimator implements Subsystem {
    private static RobotStateEstimator instance;

    public static RobotStateEstimator getInstance() {
        return instance;
    }

    public static RobotStateEstimator getInstance(BNO055IMU imu) {
        if(instance == null) {
            instance = new RobotStateEstimator(imu);
        }

        return instance;
    }

    public static RobotStateEstimator getInstance(BNO055IMU imu, Pose2d pose) {
        if(instance == null) {
            instance = new RobotStateEstimator(imu, pose);
        }

        return instance;
    }

    private BNO055IMU imu;
    private Pose2d pose;
    private Pose2d velocityPose;

    public RobotStateEstimator(BNO055IMU imu) {
        setImu(imu);
        setPose(new Pose2d());
        setVelocityPose(new Pose2d());
        initializeIMU();
    }

    public RobotStateEstimator(BNO055IMU imu, Pose2d initialPose) {
        setImu(imu);
        setPose(initialPose);
        setVelocityPose(new Pose2d());
        initializeIMU();
    }

    @Override
    public void start() {
        setPose(new Pose2d());
    }

    @Override
    public void update(double dt) {
        double backLeftVelocity = Drive.getInstance().getBackLeft().getVelocity() / getWheelRadius();
        double frontLeftVelocity = Drive.getInstance().getFrontLeft().getVelocity() / getWheelRadius();
        double backRightVelocity = Drive.getInstance().getBackRight().getVelocity() / getWheelRadius();
        double frontRightVelocity = Drive.getInstance().getFrontRight().getVelocity() / getWheelRadius();
        Rotation2d heading = getHeading();
        //Angle heading = new Angle(getImu().getAngularOrientation().firstAngle, AngularUnits.DEGREES);

        //1 - Front left
        //2 - Front right
        //3 - Back left
        //4 - Back right

        double cosPsi = heading.cos();
        double sinPsi = heading.sin();
        double denominator = (getD1() + getD2() + getL1() + getL2()) * getWheelRadius();

        //The convention here is that x is forward and y is left
        double velocityX = frontLeftVelocity * (cosPsi + sinPsi) / getWheelRadius() +
                frontRightVelocity * (-(2 * getD1() + getL1() + getL2()) * cosPsi +
                        (-getL1() + getL2()) * sinPsi) / (denominator) +
                backLeftVelocity * ((getD1() - getD2()) * cosPsi + (getD1() + getD2() + 2 * getL1())
                        * sinPsi) / denominator;
        double velocityY = frontLeftVelocity * (cosPsi - sinPsi) / getWheelRadius() +
                frontRightVelocity * ((-getL1() + getL2()) * cosPsi + (2 * getD1() + getL1() +
                        getL2()) * sinPsi) / denominator + backLeftVelocity *
                ((getD1() + getD2() + 2 * getL1()) * cosPsi + (-getD1() + getD2()) * sinPsi) / denominator;
        setVelocityPose(new Pose2d(-velocityY, velocityX, getHeading().rotateBy(getPose().getRotation().inverse()).multiply(1 / dt, false)));
        setPose(new Pose2d(getPose().getTranslation().translateBy(new Translation2d(-velocityY * dt, velocityX * dt)), heading));
        //setPose(getPose().transformBy(new Pose2d(-velocityY * dt, velocityX * dt, new Rotation2d(heading.getRadians() - getPose().getRotation().getRadians(), true))));
    }

    @Override
    public void stop() {

    }

    public DoubleMatrix getDriveState() {
        return new DoubleMatrix(6, 1, new double[] {
                getPose().getTranslation().y(),
                getVelocityPose().getTranslation().y(),
                -getPose().getTranslation().x(),
                -getVelocityPose().getTranslation().x(),
                getPose().getRotation().getRadians(),
                getVelocityPose().getRotation().getRadians()
        });
    }

    public void initializeIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile  = "BNO055IMUCalibration.json";
        parameters.loggingEnabled       = true;
        parameters.loggingTag           = "IMU";
        //parameters.accelerationIntegrationAlgorithm = new AccelerationIntegrator();

        getImu().initialize(parameters);
    }

    public Rotation2d getHeading() {
        return new Rotation2d(Math.toRadians(getImu().getAngularOrientation().firstAngle), false);
    }

    @Override
    public String toString() {
        return "Position: " + getPose();
    }

    public Pose2d getPose() {
        return pose;
    }

    public void setPose(Pose2d pose) {
        this.pose = pose;
    }

    public BNO055IMU getImu() {
        return imu;
    }

    public void setImu(BNO055IMU imu) {
        this.imu = imu;
    }

    public Pose2d getVelocityPose() {
        return velocityPose;
    }

    public void setVelocityPose(Pose2d velocityPose) {
        this.velocityPose = velocityPose;
    }
}
