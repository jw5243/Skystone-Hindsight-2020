package org.firstinspires.ftc.teamcode.ftc15026.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.ftc15026.control.StackTracker;
import org.firstinspires.ftc.teamcode.ftc15026.statemachines.FeederDumperStateMachine;
import org.firstinspires.ftc.teamcode.ftc15026.statemachines.FeederExtensionStateMachine;
import org.firstinspires.ftc.teamcode.ftc15026.statemachines.FeederRotatorStateMachine;
import org.firstinspires.ftc.teamcode.ftc15026.statemachines.FeederStopperStateMachine;
import org.firstinspires.ftc.teamcode.lib.control.ControlConstants;
import org.firstinspires.ftc.teamcode.lib.drivers.RevCRServo;
import org.firstinspires.ftc.teamcode.lib.drivers.RevServo;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.lib.motion.ResidualVibrationReductionMotionProfilerGenerator;
import org.firstinspires.ftc.teamcode.lib.motion.TrapezoidalMotionProfileGenerator;

import java.util.function.DoubleConsumer;

public class Feeder extends Extension {
    private static final double STAGE_LENGTH = 400d / 25.4d;
    private static final double EXTENSION_SETPOINT_STONE_TO_OUTTAKE = 2d;
    private static final double EXTENSION_SETPOINT_UNDER_SKYBRIDGE = 4d;

    private StackTracker stackTracker;
    private RevServo feederDumper;
    private RevServo feederStopper;
    private RevCRServo feederRotator;
    private AnalogInput potentiometer;

    private static double setpoint = 0d;
    private double rotatorSetpoint;

    public Feeder(StackTracker stackTracker, Gearbox feederExtension, RevServo feederDumper, RevServo feederStopper,
                  RevCRServo feederRotator, AnalogInput potentiometer) {
        super(feederExtension, ExtensionControl.RESIDUAL_VIBRATION_REDUCTION, new ControlConstants(
                //0.08d, 0.001d, 0d, 0.17d, 0d, 0d
                //0.02d, 0d, 0d, 0.1d, 0d, 0d
                0d, 0d, 0d, 0.2d, (1d - 0.2d) / 120d, 0d
        ));

        setStackTracker(stackTracker);
        setFeederDumper(feederDumper);
        setFeederStopper(feederStopper);
        setFeederRotator(feederRotator);
        setPotentiometer(potentiometer);
        setRotatorSetpoint(FeederRotatorStateMachine.State.INITIALIZATION.getPosition());
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void update(double dt) {
        /*int stage = (int)(getGearboxExtension().getPose() / getStageLength()) + 1;
        if(stage == 1) {
            setControlConstants(new ControlConstants(0.08d, 0d, 0.004d, 0.35d, 0d, 0d));
        } else if(stage == 2) {
            setControlConstants(new ControlConstants(0.09d, 0d, 0.006d, 0.43d, 0d, 0d));
        } else if(stage == 3) {
            setControlConstants(new ControlConstants(0.09d, 0d, 0.005d, 0.51d, 0d, 0d));
        } else if(stage == 4) {
            setControlConstants(new ControlConstants(0.09d, 0d, 0.005d, 0.59d, 0d, 0d));
        } else if(stage == 5) {
            setControlConstants(new ControlConstants(0.09d, 0d, 0.005d, 0.67d, 0d, 0d));
        } else if(stage == 6) {
            setControlConstants(new ControlConstants(0.09d, 0d, 0.005d, 0.75d, 0d, 0d));
        }*/

        if(getSetpoint() == 0d) {
            setControlConstants(new ControlConstants(
                    //0.005, 0d, 0d, 0.25d, (1d - 0.25d) / 240, 0d
                    0d, 0d, 0d, 0.25d, 0d, 0d
            ));
        } else {
            setControlConstants(new ControlConstants(
                    //0.043d, 0d, 0d, 0.2d, (1d - 0.2d) / 120d, 0.0001d
                    0d, 0d, 0d, 0.2d, 0d, 0d
            ));
        }

        if(!FeederRotatorStateMachine.getDesiredState().equals(FeederRotatorStateMachine.State.READY_FOR_LIFTOFF)) {
            FeederExtensionStateMachine.updateState(FeederExtensionStateMachine.State.IDLE);
        }

        //Check if the feeder is trying to extend.
        if(FeederExtensionStateMachine.getState().ordinal() == FeederExtensionStateMachine.State.EXTEND.ordinal()) {
            //Check to make sure that the feeder can extend without the possibility of falling into the robot.
            if(FeederRotatorStateMachine.getState().ordinal() == FeederRotatorStateMachine.State.READY_FOR_LIFTOFF.ordinal() &&
                    FeederRotatorStateMachine.hasReachedStateGoal()) {
                extend();
            }
        } else if(FeederExtensionStateMachine.getState().equals(FeederExtensionStateMachine.State.RETRACT)) {
            setSetpoint(0d);
        } else if(FeederExtensionStateMachine.getState().equals(FeederExtensionStateMachine.State.IDLE)) {
            if(FeederRotatorStateMachine.getState().equals(FeederRotatorStateMachine.State.UNDER_SKYBRIDGE)) {
                setSetpoint(4d);
            } else if(FeederRotatorStateMachine.getState().equals(FeederRotatorStateMachine.State.STONE_TO_OUTTAKE)) {
                setSetpoint(2d);
            }
        }

        //Run the usual extension profiled PID control.
        //super.update(dt);
        //Get the current error based on the position of the extension.
        double error = getSetpoint() - getGearboxExtension().getPosition();
        double velocityTarget = 0d;
        double accelerationTarget = 0d;
        if(getMotionProfile() != null) {
            error = getMotionProfile().getPosition() - getGearboxExtension().getPosition();
            velocityTarget = getMotionProfile().getVelocity();
            accelerationTarget = getMotionProfile().getAcceleration();
        }

        boolean idle = false;
        if(hasReachedDesiredExtensionLength(1 / 4d) && getSetpoint() == 0) {
            getGearboxExtension().setPower(0d);
            idle = true;
        } else if(!FeederExtensionStateMachine.getState().equals(FeederExtensionStateMachine.State.IDLE)) {
            //Update the running sum for the integral feedback controller.
            addToRunningSum(getControlConstants().kI() * error * dt);
            //Prevent integral windup
            //setRunningSum(Range.clip(getControlConstants().kI() * getRunningSum(), getMinPower(dt), getMaxPower(dt)) /
            //        getControlConstants().kI());

            double output =
                    getControlConstants().kP() * error +
                            getControlConstants().kI() * getRunningSum() +
                            getControlConstants().kD() * ((error - getLastError()) / dt - velocityTarget) +
                            getControlConstants().kV() * velocityTarget +
                            getControlConstants().kA() * accelerationTarget + getControlConstants().kS();
            getGearboxExtension().setPower(Range.clip(output, getMinPower(dt), getMaxPower(dt)));
        }

        if(idle) {
            FeederExtensionStateMachine.updateState(FeederExtensionStateMachine.State.IDLE);
        }

        setLastError(error);

        setRotatorSetpoint(FeederRotatorStateMachine.getState().getPosition());
        //getFeederDumper().setPosition(FeederDumperStateMachine.getState().getPosition());
        getFeederStopper().setPosition(FeederStopperStateMachine.getState().getPosition());

        //final double kF = 0.01d; //motor power
        final double kP = 1 / 10d; //motor power / deg
        final double kF = 0d;
        //final double kP = 0d;
        double feedforward = Math.cos(getRotatedAngle().getRadians());
        double potentiometerError = 10d * (getRotatorSetpoint() - getPotentiometer().getVoltage()); //deg
        getFeederRotator().setPower(kF * feedforward + kP * potentiometerError);
    }

    @Override
    public void stop() {
        super.stop();
        FeederExtensionStateMachine.updateState(FeederExtensionStateMachine.State.IDLE);
        getGearboxExtension().setPower(FeederExtensionStateMachine.getState().getPower());
        getFeederRotator().setPower(0d);
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
        return -1d;
    }

    @Override
    public double getMaxPower(double dt) {
        return 1d;
    }

    @Override
    public double getMaxExtensionSpeed() {
        return 20d * 6d;
    }

    @Override
    public double getMaxExtensionAcceleration() {
        return 100d * 6d;
    }

    public Rotation2d getRotatedAngle() {
        double verticalAngleReference = 10d * FeederRotatorStateMachine.State.READY_FOR_LIFTOFF.getPosition();
        return Rotation2d.fromDegrees(verticalAngleReference - 10d * getPotentiometer().getVoltage() + 90d);
    }

    public Runnable autoStack(Runnable toggleAutomation) {
        return () -> {
            FeederRotatorStateMachine.updateState(FeederRotatorStateMachine.State.READY_FOR_LIFTOFF);
            while(!FeederRotatorStateMachine.hasReachedStateGoal()) {
                try {
                    Thread.sleep(10);
                } catch(InterruptedException e) {
                    e.printStackTrace();
                }
            }

            FeederStopperStateMachine.updateState(FeederStopperStateMachine.State.BLOCKING);
            while(!FeederStopperStateMachine.hasReachedStateGoal()) {
                try {
                    Thread.sleep(10);
                } catch(InterruptedException e) {
                    e.printStackTrace();
                }
            }

            extend();
            while(!FeederExtensionStateMachine.hasReachedStateGoal()) {
                try {
                    Thread.sleep(10);
                } catch(InterruptedException e) {
                    e.printStackTrace();
                }
            }

            FeederDumperStateMachine.updateState(FeederDumperStateMachine.State.DUMP);
            while(!FeederDumperStateMachine.hasReachedStateGoal()) {
                try {
                    Thread.sleep(10);
                } catch(InterruptedException e) {
                    e.printStackTrace();
                }
            }

            FeederDumperStateMachine.updateState(FeederDumperStateMachine.State.READY_FOR_LIFTOFF);
            while(!FeederDumperStateMachine.hasReachedStateGoal()) {
                try {
                    Thread.sleep(10);
                } catch(InterruptedException e) {
                    e.printStackTrace();
                }
            }

            retract();
            while(!FeederExtensionStateMachine.hasReachedStateGoal()) {
                try {
                    Thread.sleep(10);
                } catch(InterruptedException e) {
                    e.printStackTrace();
                }
            }

            toggleAutomation.run();
        };
    }

    public RevServo getFeederDumper() {
        return feederDumper;
    }

    public void setFeederDumper(RevServo feederDumper) {
        this.feederDumper = feederDumper;
    }

    public RevCRServo getFeederRotator() {
        return feederRotator;
    }

    public void setFeederRotator(RevCRServo feederRotator) {
        this.feederRotator = feederRotator;
    }

    public double getRotatorSetpoint() {
        return rotatorSetpoint;
    }

    public void setRotatorSetpoint(double rotatorSetpoint) {
        this.rotatorSetpoint = rotatorSetpoint;
    }

    public AnalogInput getPotentiometer() {
        return potentiometer;
    }

    public void setPotentiometer(AnalogInput potentiometer) {
        this.potentiometer = potentiometer;
    }

    public static double getStageLength() {
        return STAGE_LENGTH;
    }

    public RevServo getFeederStopper() {
        return feederStopper;
    }

    public void setFeederStopper(RevServo feederStopper) {
        this.feederStopper = feederStopper;
    }

    public StackTracker getStackTracker() {
        return stackTracker;
    }

    public void setStackTracker(StackTracker stackTracker) {
        this.stackTracker = stackTracker;
    }

    @Override
    public void setSetpoint(double setpoint) {
        if(setpoint != getSetpoint() && (getMotionProfile() == null || getMotionProfile().isDone())) {
            if(setpoint != 0d) {
                //Extending
                if(getExtensionControl().equals(ExtensionControl.TRAPEZOIDAL)) {
                    setMotionProfile(new TrapezoidalMotionProfileGenerator(setpoint, getGearboxExtension().getPosition(), getGearboxExtension().getVelocity(), getMaxExtensionSpeed(), getMaxExtensionAcceleration()));
                    getMotionProfile().start();
                } else if(getExtensionControl().equals(ExtensionControl.RESIDUAL_VIBRATION_REDUCTION)) {
                    if(setpoint == getExtensionSetpointStoneToOuttake()) {
                        setMotionProfile(ResidualVibrationReductionMotionProfilerGenerator.getFeederStoneToOuttakeProfileExtend());
                    } else if(setpoint == getExtensionSetpointUnderSkybridge()) {
                        setMotionProfile(ResidualVibrationReductionMotionProfilerGenerator.getFeederUnderSkybridgeProfileExtend());
                    } else {
                        setMotionProfile(getStackTracker().motionProfilerSetpoints(true));
                    }

                    getMotionProfile().start();
                }
            } else {
                //Retracting
                if(getExtensionControl().equals(ExtensionControl.TRAPEZOIDAL)) {
                    setMotionProfile(new TrapezoidalMotionProfileGenerator(0d, getGearboxExtension().getPosition(), getGearboxExtension().getVelocity(), getMaxExtensionSpeed(), getMaxExtensionAcceleration()));
                    getMotionProfile().start();
                } else if(getExtensionControl().equals(ExtensionControl.RESIDUAL_VIBRATION_REDUCTION)) {
                    if(setpoint == getExtensionSetpointStoneToOuttake()) {
                        setMotionProfile(ResidualVibrationReductionMotionProfilerGenerator.getFeederStoneToOuttakeProfileRetract());
                    } else if(setpoint == getExtensionSetpointUnderSkybridge()) {
                        setMotionProfile(ResidualVibrationReductionMotionProfilerGenerator.getFeederUnderSkybridgeProfileRetract());
                    } else {
                        setMotionProfile(getStackTracker().motionProfilerSetpoints(false));
                    }

                    if(getMotionProfile() != null) {
                        getMotionProfile().start();
                    }
                }
            }

            Feeder.setpoint = setpoint;
        }
    }

    public void extend() {
        setSetpoint(getStackTracker().getExtensionHeight());
    }

    public static double getExtensionSetpointStoneToOuttake() {
        return EXTENSION_SETPOINT_STONE_TO_OUTTAKE;
    }

    public static double getExtensionSetpointUnderSkybridge() {
        return EXTENSION_SETPOINT_UNDER_SKYBRIDGE;
    }
}
