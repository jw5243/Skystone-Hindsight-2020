package org.firstinspires.ftc.teamcode.ftc10515;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.ftc10515.control.StackTracker;
import org.firstinspires.ftc.teamcode.ftc10515.subsystems.Drive;
import org.firstinspires.ftc.teamcode.ftc10515.subsystems.ExpansionHubs;
import org.firstinspires.ftc.teamcode.ftc10515.subsystems.Feeder;
import org.firstinspires.ftc.teamcode.ftc10515.subsystems.FlywheelCollector;
import org.firstinspires.ftc.teamcode.ftc10515.subsystems.FoundationClaws;
import org.firstinspires.ftc.teamcode.ftc10515.subsystems.Gearbox;
import org.firstinspires.ftc.teamcode.ftc10515.subsystems.RobotStateEstimator;
import org.firstinspires.ftc.teamcode.ftc10515.subsystems.Superstructure;
import org.firstinspires.ftc.teamcode.lib.drivers.Motor;
import org.firstinspires.ftc.teamcode.lib.drivers.RevCRServo;
import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;
import org.firstinspires.ftc.teamcode.lib.drivers.RevServo;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;

public abstract class SkystoneRobot extends Robot {
    private static ExpansionHubs expansionHubs;
    private static Superstructure superstructure;
    private static RobotStateEstimator robotStateEstimator;
    private static Drive drive;
    private static FlywheelCollector flywheelCollector;
    private static Feeder feeder;
    private static FoundationClaws foundationClaws;

    private static StackTracker stackTracker;

    @Override
    public void init() {
        super.init();
        setExpansionHubs(ExpansionHubs.getInstance(
                hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1"),
                hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2"))
        );

        setMotors(new RevMotor[] {
                new RevMotor((ExpansionHubMotor)(hardwareMap.get("RL")), true, true, true, false, Motor.GOBILDA_435_RPM.getENCODER_TICKS_PER_REVOLUTION(), getWheelDiameter(), 0.5d),
                new RevMotor((ExpansionHubMotor)(hardwareMap.get("FL")), true, true, true, false, Motor.GOBILDA_435_RPM.getENCODER_TICKS_PER_REVOLUTION(), getWheelDiameter(), 0.5d),
                new RevMotor((ExpansionHubMotor)(hardwareMap.get("RR")), false, true, true, true, Motor.GOBILDA_435_RPM.getENCODER_TICKS_PER_REVOLUTION(), getWheelDiameter(), 0.5d),
                new RevMotor((ExpansionHubMotor)(hardwareMap.get("FR")), false, true, true, true, Motor.GOBILDA_435_RPM.getENCODER_TICKS_PER_REVOLUTION(), getWheelDiameter(), 0.5d),
                new RevMotor((ExpansionHubMotor)(hardwareMap.get("LL")), true, true, false, true, Motor.GOBILDA_312_RPM.getENCODER_TICKS_PER_REVOLUTION(), 2d),
                new RevMotor((ExpansionHubMotor)(hardwareMap.get("LR")), false, true, false, false, Motor.GOBILDA_312_RPM.getENCODER_TICKS_PER_REVOLUTION(), 2d),
                new RevMotor((ExpansionHubMotor)(hardwareMap.get("INR")), false, false, false, false),
                new RevMotor((ExpansionHubMotor)(hardwareMap.get("INL")), true, false, false, false),
        });

        setServos(new RevServo[] {
                null,//new RevServo((ExpansionHubServo)(hardwareMap.get("feederClaw"))),
                new RevServo((ExpansionHubServo)(hardwareMap.get("FSL"))),
                new RevServo((ExpansionHubServo)(hardwareMap.get("FSR")))
        });

        setCrServos(null);

        setSuperstructure(Superstructure.getInstance());
        setRobotStateEstimator(RobotStateEstimator.getInstance(hardwareMap.get(BNO055IMU.class, "imu"), new Pose2d()));
        setDrive(Drive.getInstance(getMotors()[0], getMotors()[1], getMotors()[2], getMotors()[3]));
        setFlywheelCollector(FlywheelCollector.getInstance(getMotors()[7], getMotors()[6]));
        setFeeder(Feeder.getInstance(getMotors()[4], getMotors()[5], getServos()[0]));
        setFoundationClaws(FoundationClaws.getInstance(getServos()[1], getServos()[2]));
        setStackTracker(StackTracker.getInstance());
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
        getExpansionHubs().start();
        getSuperstructure().start();
        getRobotStateEstimator().start();
        getDrive().start();
        getFlywheelCollector().start();
        getFeeder().start();
    }

    @Override
    public void loop() {
        super.loop();
        getExpansionHubs().update(getDt());
        getSuperstructure().update(getDt());
        getRobotStateEstimator().update(getDt());
        getDrive().update(getDt());
        getFlywheelCollector().update(getDt());
        getFeeder().update(getDt());
    }

    @Override
    public void stop() {
        super.stop();
        getExpansionHubs().stop();
        getSuperstructure().stop();
        getRobotStateEstimator().stop();
        getDrive().stop();
        getFlywheelCollector().stop();
        getFeeder().stop();
    }

    public static Superstructure getSuperstructure() {
        return superstructure;
    }

    public static void setSuperstructure(Superstructure superstructure) {
        SkystoneRobot.superstructure = superstructure;
    }

    public static RobotStateEstimator getRobotStateEstimator() {
        return robotStateEstimator;
    }

    public static void setRobotStateEstimator(RobotStateEstimator robotStateEstimator) {
        SkystoneRobot.robotStateEstimator = robotStateEstimator;
    }

    public static Drive getDrive() {
        return drive;
    }

    public static void setDrive(Drive drive) {
        SkystoneRobot.drive = drive;
    }

    public static FlywheelCollector getFlywheelCollector() {
        return flywheelCollector;
    }

    public static void setFlywheelCollector(FlywheelCollector flywheelCollector) {
        SkystoneRobot.flywheelCollector = flywheelCollector;
    }

    public static Feeder getFeeder() {
        return feeder;
    }

    public static void setFeeder(Feeder feeder) {
        SkystoneRobot.feeder = feeder;
    }

    public static ExpansionHubs getExpansionHubs() {
        return expansionHubs;
    }

    public static void setExpansionHubs(ExpansionHubs expansionHubs) {
        SkystoneRobot.expansionHubs = expansionHubs;
    }

    public static StackTracker getStackTracker() {
        return stackTracker;
    }

    public static void setStackTracker(StackTracker stackTracker) {
        SkystoneRobot.stackTracker = stackTracker;
    }

    public static FoundationClaws getFoundationClaws() {
        return foundationClaws;
    }

    public static void setFoundationClaws(FoundationClaws foundationClaws) {
        SkystoneRobot.foundationClaws = foundationClaws;
    }
}
