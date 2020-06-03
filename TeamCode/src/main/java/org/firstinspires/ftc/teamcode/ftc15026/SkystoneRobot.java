package org.firstinspires.ftc.teamcode.ftc15026;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.ftc15026.control.StackTracker;
import org.firstinspires.ftc.teamcode.ftc15026.statemachines.CollectorDumperStateMachine;
import org.firstinspires.ftc.teamcode.ftc15026.statemachines.CollectorExtensionStateMachine;
import org.firstinspires.ftc.teamcode.ftc15026.statemachines.FeederDumperStateMachine;
import org.firstinspires.ftc.teamcode.ftc15026.statemachines.FeederExtensionStateMachine;
import org.firstinspires.ftc.teamcode.ftc15026.statemachines.FeederRotatorStateMachine;
import org.firstinspires.ftc.teamcode.ftc15026.statemachines.FeederStopperStateMachine;
import org.firstinspires.ftc.teamcode.ftc15026.statemachines.SpindleStateMachine;
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
import org.firstinspires.ftc.teamcode.lib.motion.ResidualVibrationReductionMotionProfilerGenerator;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;

import java.util.Arrays;

public abstract class SkystoneRobot extends Robot {
    private ExpansionHubs       expansionHubs;
    private Superstructure      superstructure;
    private RobotStateEstimator robotStateEstimator;
    private Drive               drive;
    private Collector           collector;
    private Feeder              feeder;
    private StackTracker        stackTracker;

    @Override
    public void init() {
        super.init();
        ResidualVibrationReductionMotionProfilerGenerator.init();
        setExpansionHubs(new ExpansionHubs(this,
                hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1"),
                hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2"))
        );

        setMotors(new RevMotor[] {
                new RevMotor((ExpansionHubMotor)(hardwareMap.get("backLeft")), true, true, true, true, Motor.GOBILDA_223_RPM.getENCODER_TICKS_PER_REVOLUTION(), getWheelDiameter()),
                new RevMotor((ExpansionHubMotor)(hardwareMap.get("frontLeft")), true, true, true, true, Motor.GOBILDA_223_RPM.getENCODER_TICKS_PER_REVOLUTION(), getWheelDiameter()),
                new RevMotor((ExpansionHubMotor)(hardwareMap.get("backRight")), true, true, true, false, Motor.GOBILDA_223_RPM.getENCODER_TICKS_PER_REVOLUTION(), getWheelDiameter()),
                new RevMotor((ExpansionHubMotor)(hardwareMap.get("frontRight")), true, true, true, false, Motor.GOBILDA_223_RPM.getENCODER_TICKS_PER_REVOLUTION(), getWheelDiameter()),
                new RevMotor((ExpansionHubMotor)(hardwareMap.get("collectorExtension")), false, true, false, false, Motor.GOBILDA_312_RPM.getENCODER_TICKS_PER_REVOLUTION(), 2.25d),
                new RevMotor((ExpansionHubMotor)(hardwareMap.get("spindle")), false, false, false, true),
                new RevMotor((ExpansionHubMotor)(hardwareMap.get("feederExtension1")), false, true, false, true, Motor.GOBILDA_223_RPM.getENCODER_TICKS_PER_REVOLUTION(), 2.25d * 6d),
                new RevMotor((ExpansionHubMotor)(hardwareMap.get("feederExtension2")), false, true, false, true, Motor.GOBILDA_223_RPM.getENCODER_TICKS_PER_REVOLUTION(), 2.25d * 6d)
        });

        setServos(new RevServo[] {
                new RevServo((ExpansionHubServo)(hardwareMap.get("collectorDumper"))),
                //new RevServo((ExpansionHubServo)(hardwareMap.get("feederDumper"))),
                new RevServo((ExpansionHubServo)(hardwareMap.get("feederStopper")))
        });

        setCrServos(new RevCRServo[] {
                new RevCRServo(hardwareMap.get(CRServo.class, "feederRotator"))
        });

        setSuperstructure(new Superstructure());
        setRobotStateEstimator(new RobotStateEstimator(this, hardwareMap.get(BNO055IMU.class, "imu"), new Pose2d()));
        setDrive(new Drive(getRobotStateEstimator(), getMotors()[0], getMotors()[1], getMotors()[2], getMotors()[3]));
        setCollector(new Collector(getMotors()[4], getMotors()[5], getServos()[0]));
        setStackTracker(new StackTracker());
        setFeeder(new Feeder(getStackTracker(), new Gearbox(getMotors()[6], getMotors()[7]), /*getServos()[1]*/null,
                getServos()[1], getCrServos()[0], hardwareMap.get(AnalogInput.class, "potentiometer")));
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
        CollectorDumperStateMachine.init(this);
        CollectorExtensionStateMachine.init(this);
        FeederDumperStateMachine.init(this);
        FeederExtensionStateMachine.init(this);
        FeederRotatorStateMachine.init(this);
        FeederStopperStateMachine.init(this);
        SpindleStateMachine.init(this);

        getExpansionHubs().start();
        getSuperstructure().start();
        getRobotStateEstimator().start();
        getDrive().start();
        getCollector().start();
        getFeeder().start();
        Arrays.stream(getMotors()).forEach(RevMotor::resetEncoder);
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
        FeederStopperStateMachine.update();
        FeederRotatorStateMachine.update();
        SpindleStateMachine.update();
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

    public ExpansionHubs getExpansionHubs() {
        return expansionHubs;
    }

    public void setExpansionHubs(ExpansionHubs expansionHubs) {
        this.expansionHubs = expansionHubs;
    }

    public Superstructure getSuperstructure() {
        return superstructure;
    }

    public void setSuperstructure(Superstructure superstructure) {
        this.superstructure = superstructure;
    }

    public RobotStateEstimator getRobotStateEstimator() {
        return robotStateEstimator;
    }

    public void setRobotStateEstimator(RobotStateEstimator robotStateEstimator) {
        this.robotStateEstimator = robotStateEstimator;
    }

    public Drive getDrive() {
        return drive;
    }

    public void setDrive(Drive drive) {
        this.drive = drive;
    }

    public Collector getCollector() {
        return collector;
    }

    public void setCollector(Collector collector) {
        this.collector = collector;
    }

    public Feeder getFeeder() {
        return feeder;
    }

    public void setFeeder(Feeder feeder) {
        this.feeder = feeder;
    }

    public StackTracker getStackTracker() {
        return stackTracker;
    }

    public void setStackTracker(StackTracker stackTracker) {
        this.stackTracker = stackTracker;
    }

    public Pose2d getRobotPose() {
        return getRobotStateEstimator().getPose();
    }

    public double getRobotSpeed() {
        return getRobotStateEstimator().getVelocityPose().getTranslation().norm() +
                Math.abs(getRobotStateEstimator().getVelocityPose().getRotation().getRadians());
    }
}
