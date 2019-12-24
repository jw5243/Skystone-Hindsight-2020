package org.firstinspires.ftc.teamcode.ftc15026;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.ftc15026.subsystems.Collector;
import org.firstinspires.ftc.teamcode.ftc15026.subsystems.Drive;
import org.firstinspires.ftc.teamcode.ftc15026.subsystems.ExpansionHubs;
import org.firstinspires.ftc.teamcode.ftc15026.subsystems.Feeder;
import org.firstinspires.ftc.teamcode.ftc15026.subsystems.Gearbox;
import org.firstinspires.ftc.teamcode.ftc15026.subsystems.RobotStateEstimator;
import org.firstinspires.ftc.teamcode.ftc15026.subsystems.Superstructure;
import org.firstinspires.ftc.teamcode.lib.drivers.Motor;
import org.firstinspires.ftc.teamcode.lib.drivers.RevCRServo;
import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;
import org.firstinspires.ftc.teamcode.lib.drivers.RevServo;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;

public abstract class SkystoneRobot extends Robot {
    private static ExpansionHubs       expansionHubs;
    private static Superstructure      superstructure;
    private static RobotStateEstimator robotStateEstimator;
    private static Drive               drive;
    private static Collector           collector;
    private static Feeder              feeder;

    @Override
    public void init() {
        super.init();
        setExpansionHubs(ExpansionHubs.getInstance(
                hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1"),
                hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2"))
        );

        setMotors(new RevMotor[] {
                new RevMotor((ExpansionHubMotor)(hardwareMap.get("backLeft")), true, true, true, false, Motor.GOBILDA_223_RPM.getENCODER_TICKS_PER_REVOLUTION(), getWheelDiameter()),
                new RevMotor((ExpansionHubMotor)(hardwareMap.get("frontLeft")), true, true, true, false, Motor.GOBILDA_223_RPM.getENCODER_TICKS_PER_REVOLUTION(), getWheelDiameter()),
                new RevMotor((ExpansionHubMotor)(hardwareMap.get("backRight")), true, true, true, true, Motor.GOBILDA_223_RPM.getENCODER_TICKS_PER_REVOLUTION(), getWheelDiameter()),
                new RevMotor((ExpansionHubMotor)(hardwareMap.get("frontRight")), true, true, true, true, Motor.GOBILDA_223_RPM.getENCODER_TICKS_PER_REVOLUTION(), getWheelDiameter()),
                new RevMotor((ExpansionHubMotor)(hardwareMap.get("collectorExtension")), false, true, false, false, Motor.GOBILDA_312_RPM.getENCODER_TICKS_PER_REVOLUTION(), 2.25d),
                new RevMotor((ExpansionHubMotor)(hardwareMap.get("spindle")), false, false, false, false),
                new RevMotor((ExpansionHubMotor)(hardwareMap.get("feederExtension1")), false, true, false, false, Motor.NEVERST_3_7.getENCODER_TICKS_PER_REVOLUTION(), 2.25d),
                new RevMotor((ExpansionHubMotor)(hardwareMap.get("feederExtension2")), false, true, false, false, Motor.NEVERST_3_7.getENCODER_TICKS_PER_REVOLUTION(), 2.25d)
        });

        setServos(new RevServo[] {
                new RevServo((ExpansionHubServo)(hardwareMap.get("collectorDumper"))),
                new RevServo((ExpansionHubServo)(hardwareMap.get("feederDumper"))),
        });

        setCrServos(new RevCRServo[] {
                new RevCRServo(hardwareMap.get(CRServo.class, "feederRotator"))
        });

        setSuperstructure(Superstructure.getInstance());
        setRobotStateEstimator(RobotStateEstimator.getInstance(hardwareMap.get(BNO055IMU.class, "imu"), new Pose2d()));
        setDrive(Drive.getInstance(getMotors()[0], getMotors()[1], getMotors()[2], getMotors()[3]));
        setCollector(Collector.getInstance(getMotors()[4], getMotors()[5], getServos()[0]));
        setFeeder(Feeder.getInstance(new Gearbox(getMotors()[6], getMotors()[7]), getServos()[1], getCrServos()[0]));
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
        getCollector().start();
        getFeeder().start();
    }

    @Override
    public void loop() {
        super.loop();
        getExpansionHubs().update(getDt());
        getSuperstructure().update(getDt());
        getRobotStateEstimator().update(getDt());
        getDrive().update(getDt());
        getCollector().update(getDt());
        getFeeder().update(getDt());
    }

    @Override
    public void stop() {
        super.stop();
        getExpansionHubs().stop();
        getSuperstructure().stop();
        getRobotStateEstimator().stop();
        getDrive().stop();
        getCollector().stop();
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

    public static Collector getCollector() {
        return collector;
    }

    public static void setCollector(Collector collector) {
        SkystoneRobot.collector = collector;
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
}
