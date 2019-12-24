package org.firstinspires.ftc.teamcode.ftc15026;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.ftc15026.control.EnhancedGamepad;
import org.firstinspires.ftc.teamcode.lib.drivers.RevCRServo;
import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;
import org.firstinspires.ftc.teamcode.lib.drivers.RevServo;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;

public abstract class Robot extends OpMode {
    private static final boolean isUsingComputer = false;

    private static final double WHEEL_DIAMETER = 100d / 25.4d;
    private static final double L1 = 9d;
    private static final double L2 = 9d;
    private static final double D1 = 9d;
    private static final double D2 = 9d;
    private static final double ROBOT_LENGTH = getL1() + getL2();

    private static Alliance alliance;
    private static Pose2d   drivetrainPower;

    private static EnhancedGamepad enhancedGamepad1;
    private static EnhancedGamepad enhancedGamepad2;

    private static RevMotor[]   motors;
    private static RevServo[]   servos;
    private static RevCRServo[] crServos;

    private static TimeProfiler updateRuntime;
    private static double dt;

    @Override
    public void init() {
        setAlliance(Alliance.BLUE);
        setDrivetrainPower(Pose2d.identity());
        setEnhancedGamepad1(new EnhancedGamepad(gamepad1));
        setEnhancedGamepad2(new EnhancedGamepad(gamepad2));
        setUpdateRuntime(new TimeProfiler(false));
    }

    @Override
    public void init_loop() {
        getEnhancedGamepad1().update();
        getEnhancedGamepad2().update();
        if(getEnhancedGamepad1().isxJustPressed()) {
            setAlliance(getAlliance().getOpposingAlliance());
        }

        telemetry.addLine(getAlliance().toString());
        telemetry.update();
    }

    @Override
    public void start() {
        getUpdateRuntime().start();
    }

    @Override
    public void loop() {
        setDt(getUpdateRuntime().getDeltaTime(TimeUnits.SECONDS, true));
        getEnhancedGamepad1().update();
        getEnhancedGamepad2().update();
    }

    public static boolean isUsingComputer() {
        return isUsingComputer;
    }

    public static Alliance getAlliance() {
        return alliance;
    }

    public static void setAlliance(Alliance alliance) {
        Robot.alliance = alliance;
    }

    public static EnhancedGamepad getEnhancedGamepad1() {
        return enhancedGamepad1;
    }

    public static void setEnhancedGamepad1(EnhancedGamepad enhancedGamepad1) {
        Robot.enhancedGamepad1 = enhancedGamepad1;
    }

    public static EnhancedGamepad getEnhancedGamepad2() {
        return enhancedGamepad2;
    }

    public static void setEnhancedGamepad2(EnhancedGamepad enhancedGamepad2) {
        Robot.enhancedGamepad2 = enhancedGamepad2;
    }

    public static RevMotor[] getMotors() {
        return motors;
    }

    public static void setMotors(RevMotor[] motors) {
        Robot.motors = motors;
    }

    public static RevServo[] getServos() {
        return servos;
    }

    public static void setServos(RevServo[] servos) {
        Robot.servos = servos;
    }

    public static TimeProfiler getUpdateRuntime() {
        return updateRuntime;
    }

    public static void setUpdateRuntime(TimeProfiler updateRuntime) {
        Robot.updateRuntime = updateRuntime;
    }

    public static double getDt() {
        return dt;
    }

    public static void setDt(double dt) {
        Robot.dt = dt;
    }

    public static Pose2d getDrivetrainPower() {
        return drivetrainPower;
    }

    public static void setDrivetrainPower(Pose2d drivetrainPower) {
        Robot.drivetrainPower = drivetrainPower;
    }

    public static double getL1() {
        return L1;
    }

    public static double getL2() {
        return L2;
    }

    public static double getD1() {
        return D1;
    }

    public static double getD2() {
        return D2;
    }

    public static double getRobotLength() {
        return ROBOT_LENGTH;
    }

    public static double getWheelDiameter() {
        return WHEEL_DIAMETER;
    }

    public static double getWheelRadius() {
        return getWheelDiameter() / 2d;
    }

    public static RevCRServo[] getCrServos() {
        return crServos;
    }

    public static void setCrServos(RevCRServo[] crServos) {
        Robot.crServos = crServos;
    }
}
