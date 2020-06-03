package org.firstinspires.ftc.teamcode.ftc15026.subsystems;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.ftc15026.statemachines.CollectorDumperStateMachine;
import org.firstinspires.ftc.teamcode.ftc15026.statemachines.CollectorExtensionStateMachine;
import org.firstinspires.ftc.teamcode.ftc15026.statemachines.FeederRotatorStateMachine;
import org.firstinspires.ftc.teamcode.ftc15026.statemachines.SpindleStateMachine;
import org.firstinspires.ftc.teamcode.lib.control.ControlConstants;
import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;
import org.firstinspires.ftc.teamcode.lib.drivers.RevServo;
import org.firstinspires.ftc.teamcode.lib.util.TimeProfiler;

import java.util.function.DoubleConsumer;

public class Collector extends Extension {
    private static final double MAX_EXTENSION_LENGTH = 10d;//25d; //in
    private static final double MAX_COLLECTOR_EXTENSION_SPEED = 20d; //in/s
    private static final double COLLECTOR_DUMPER_ROTATE_THRESHOLD = 3d; //in
    private static final double ACCELERATION_CAP = 1d; //motor power / s

    private RevMotor spindle;
    private RevServo collectorDumper;
    private double lastPower;

    private static double setpoint = 0d;

    public Collector(RevMotor collectorExtension, RevMotor spindle, RevServo collectorDumper) {
        //The collector does not move to specific setpoints predetermined by the robot itself,
        //so instead we build a realtime trapezoidal profile by limiting the acceleration possible
        //by this mechanism. Such is done not by the encoders, but by the power of the collector
        //extension motor.
        super(collectorExtension, ExtensionControl.NONE, new ControlConstants(
                0.04d/*0.10d*/, 0.03d, 0.01d, 0.12d/*0.15d*//*0.21d*/, 0d, 0d
        ));

        setLastPower(0d);
        setSpindle(spindle);
        setCollectorDumper(collectorDumper);
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void update(double dt) {
        super.update(dt);
        //Set collector dumper position in the robot or on the ground based on the drawer slide extension position.
        //Only do this if the intake is currently trying to extend or retract, to ensure that if the
        //drawer slides extend due to the drivetrain's momentum, the container will not unnecessarily
        //flip up and down.
        //if(CollectorExtensionStateMachine.getState().ordinal() != CollectorExtensionStateMachine.State.IDLE.ordinal()) {
            CollectorDumperStateMachine.updateState(
                    getGearboxExtension().getPosition() > getCollectorDumperRotateThreshold() ?
                            CollectorDumperStateMachine.State.ON_GROUND : CollectorDumperStateMachine.State.IN_ROBOT);
        //}

        if(CollectorExtensionStateMachine.getState().equals(CollectorExtensionStateMachine.State.EXTEND)) {
            setSetpoint(getSetpoint() + getMaxCollectorExtensionSpeed() * dt);
        } else if(CollectorExtensionStateMachine.getState().equals(CollectorExtensionStateMachine.State.RETRACT)) {
            setSetpoint(getSetpoint() - getMaxCollectorExtensionSpeed() * dt);
        } else {
            setSetpoint(getSetpoint());
        }

        getCollectorDumper().setPosition(CollectorDumperStateMachine.getState().getPosition());
        getSpindle().setPower(SpindleStateMachine.getState().getPower());

        setLastPower(getGearboxExtension().getPower());
    }

    @Override
    public void stop() {
        super.stop();
        CollectorExtensionStateMachine.updateState(CollectorExtensionStateMachine.State.IDLE);
        SpindleStateMachine.updateState(SpindleStateMachine.State.IDLE);
        getGearboxExtension().setPower(CollectorExtensionStateMachine.getState().getPower());
        getSpindle().setPower(SpindleStateMachine.getState().getPower());
    }

    @Override
    public double getSetpoint() {
        return setpoint;
    }

    @Override
    public DoubleConsumer updateSetpointUnprotected() {
        return (updatedSetpoint) -> setpoint = updatedSetpoint;
    }

    @Override
    public double getMinPower(double dt) {
        return getLastPower() - getMaxExtensionAcceleration() * dt;
    }

    @Override
    public double getMaxPower(double dt) {
        return getLastPower() + getMaxExtensionAcceleration() * dt;
    }

    @Override
    public double getMaxExtensionSpeed() {
        return 10d;
    }

    @Override
    public double getMaxExtensionAcceleration() {
        return 200d;
    }

    public Runnable autoCollect(double setpoint) {
        return () -> {
            setSetpoint(setpoint);
            while(!CollectorExtensionStateMachine.hasReachedStateGoal()) {
                update(0d);
            }

            retract();
            while(!CollectorExtensionStateMachine.hasReachedStateGoal()) {
                update(0d);
            }
        };
    }

    public RevMotor getSpindle() {
        return spindle;
    }

    public void setSpindle(RevMotor spindle) {
        this.spindle = spindle;
    }

    public RevServo getCollectorDumper() {
        return collectorDumper;
    }

    public void setCollectorDumper(RevServo collectorDumper) {
        this.collectorDumper = collectorDumper;
    }

    public static double getCollectorDumperRotateThreshold() {
        return COLLECTOR_DUMPER_ROTATE_THRESHOLD;
    }

    public static double getMaxCollectorExtensionSpeed() {
        return MAX_COLLECTOR_EXTENSION_SPEED;
    }

    @Override
    public void setSetpoint(double setpoint) {
        super.setSetpoint(Range.clip(setpoint, FeederRotatorStateMachine.checkStateFinished(FeederRotatorStateMachine.State.READY_FOR_LIFTOFF) ||
                FeederRotatorStateMachine.getState().equals(FeederRotatorStateMachine.State.STONE_TO_OUTTAKE)
                ? 0d : getCollectorDumperRotateThreshold() + 1d, getMaxExtensionLength()));
    }

    public double getLastPower() {
        return lastPower;
    }

    public void setLastPower(double lastPower) {
        this.lastPower = lastPower;
    }

    public static double getAccelerationCap() {
        return ACCELERATION_CAP;
    }

    public static double getMaxExtensionLength() {
        return MAX_EXTENSION_LENGTH;
    }
}
