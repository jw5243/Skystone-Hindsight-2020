package org.firstinspires.ftc.teamcode.ftc15026.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.lib.control.ControlConstants;
import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;
import org.firstinspires.ftc.teamcode.lib.motion.IMotionProfile;
import org.firstinspires.ftc.teamcode.lib.motion.ResidualVibrationReductionMotionProfilerGenerator;
import org.firstinspires.ftc.teamcode.lib.motion.TrapezoidalMotionProfileGenerator;

import java.util.function.DoubleConsumer;

/**
 * This {@code class} represents a linear extension mechanism that is run by a motor gearbox. This
 * extension process must be controlled, so this {@code subsystem} takes care of that with feedback
 * and feedforward controllers to follow a given trajectory towards a setpoint.
 *
 * The feedback controller consists of a standard PID loop to stabilize the linear extension around
 * the reference trajectory's current desired position. The feedforward controller on the other hand,
 * takes into account the reference trajectory's desired velocity and acceleration. There is an
 * additional feedforward controller that attempts to overcome static friction in the system, so
 * that an integral term in the feedback controller is essentially unnecessary.
 *
 * We consider a linear extension as a {@code subsystem} as an extension mechanism is most often
 * coordinated with an additional mechanism to complete an overall task in conjunction.
 *
 * @see IMotionProfile
 */
public abstract class Extension implements Subsystem {
    /**
     * This represents the aggregate of motors for running the linear extension. It is assumed that
     * the motors comprising the gearbox run in the same direction, or motors are set in reverse if
     * necessary.
     *
     * @see RevMotor
     * @see Direction#REVERSE
     */
    private Gearbox gearboxExtension;

    /**
     * This contains all of the constants for every possible feedback and feedforward controller
     * that this {@code class} can supply. While not every controller will be used, different
     * constructors and simply setting unused constants to zero notifies the system not to use the
     * corresponding controllers.
     */
    private ControlConstants controlConstants;
    private ExtensionControl extensionControl;
    private IMotionProfile motionProfile;
    private double lastError;
    private double runningSum;

    public enum ExtensionControl {
        NONE,

        /**
         * This controller is the most basic form of motion profiling, which simply yields the max
         * power of the motor(s) based on the sign of the error of the extension position, so max
         * power to reach the goal, and negative of the max power to compensate for overshoot. This
         * is not usually an effective controller as overshoot is inevitable with this controller,
         * as it assumes infinite acceleration is possible once it reaches the goal state, but in
         * actuality the controlled motor slowly (as in, not immediately, but it is actually quite
         * quick) drops to no power. A goal threshold can try to remove this overshoot-oscillation,
         * but the end position will be farther than the goal, unless a slip prediction is created.
         */
        BANG_BANG,

        /**
         * This is one of the most common motion profiles used that can reach the goal smoothly.
         * By knowing the max speed and acceleration of motor(s), we can have them accelerate at
         * this max acceleration to reach max speed, and decelerate back down to zero so the robot
         * has smoothly reached its goal. Aside from bang-bang control, this method is typically
         * the fastest controlled method to reach a goal, but it comes with some downsides. First
         * of all, once the robot reaches max speed, the robot cannot accelerate anymore (positively),
         * so the robot must go from max acceleration to no acceleration in an instant, causing a
         * theoretical infinite amount of jerk in the system at that point (same happens at the
         * transition between the cruising state to decelerating state). While this can be a major
         * issue if high precision and consistency is required, but in many cases a basic control
         * system like this that constrains the velocity curve to be smooth everywhere except the
         * transition points suffices.
         */
        TRAPEZOIDAL,

        /**
         * We can go further than constraining just the velocity curve to be mostly smooth. We can
         * make the velocity profile be completely differentiable (and thus the acceleration profile
         * is continuous), by specifying a max amount of jerk in the system. This is similar to the
         * trapezoidal profile in that it makes use of the next derivative's maximum capabilities to
         * generate the fastest profile possible with the constrained motion given, in this case
         * constraining acceleration. This method tends to be slightly slower than a trapezoidal
         * profile, but results in much smoother motion, as the acceleration profile has no
         * discontinuities.
         */
        S_CURVE,

        /**
         * Sometimes, complete control of the system is necessary. That is, in the case of linear
         * extension, we need to constrain the motion to reduce wobbling of the system, or its
         * vibration, since the system may become unstable if shaken too much. While an s-curve is
         * indeed fairly smooth, to reduce vibrations, a jerk-constrained profile is required. The
         * jerk in the system causes it to vibrate. In the case of the Skystone FTC challenge,
         * stacking stones requires high precision since we cannot rely on waiting for the system to
         * stabilize after several seconds. This method of motion profiling is generally the slowest
         * out of all the other methods mentioned here, but this method provides the greatest amount
         * of control.
         */
        RESIDUAL_VIBRATION_REDUCTION,

        /**
         *
         */
        FAST_RESIDUAL_VIBRATION_REDUCTION
    }

    public Extension(RevMotor motor) {
        this(new Gearbox(motor));
    }

    public Extension(RevMotor motor1, RevMotor motor2) {
        this(new Gearbox(motor1, motor2));
    }

    public Extension(Gearbox gearbox) {
        this(gearbox, ExtensionControl.BANG_BANG, new ControlConstants());
    }

    public Extension(RevMotor motor, ExtensionControl extensionControl, ControlConstants controlConstants) {
        this(new Gearbox(motor), extensionControl, controlConstants);
    }

    public Extension(Gearbox gearbox, ExtensionControl extensionControl, ControlConstants controlConstants) {
        setGearboxExtension(gearbox);
        setControlConstants(controlConstants);
        setExtensionControl(extensionControl);
        resetRunningSum();
        setLastError(0d);
    }

    @Override
    public void start() {

    }

    /**
     * Based on the {@code setpoint} specified for the extension to reach, the motor gearbox
     * compensates for potential error for a given motion profile, otherwise only a feedback
     * controller is used.
     *
     * @param dt Elapsed time since {@code update} was last called.
     */
    @Override
    public void update(double dt) {
        //Get the current error based on the position of the extension.
        double error = getSetpoint() - getGearboxExtension().getPosition();
        double velocityTarget = 0d;
        double accelerationTarget = 0d;
        if(getMotionProfile() != null) {
            error = getMotionProfile().getPosition() - getGearboxExtension().getPosition();
            velocityTarget = getMotionProfile().getVelocity();
            accelerationTarget = getMotionProfile().getAcceleration();
        }

        if(!hasReachedDesiredExtensionLength(1 / 4d)) {
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
                            getControlConstants().kA() * accelerationTarget;
            getGearboxExtension().setPower(Range.clip(output + getControlConstants().kS() * Math.signum(output), getMinPower(dt), getMaxPower(dt)));
        } else {
            getGearboxExtension().setPower(0d);
        }

        setLastError(error);
    }

    @Override
    public void stop() {
        getGearboxExtension().setPower(0d);
    }

    public boolean hasReachedDesiredExtensionLength(double threshold) {
        return Math.abs(getSetpoint() - getGearboxExtension().getPosition()) <= threshold;
    }

    public abstract double getSetpoint();
    public abstract DoubleConsumer updateSetpointUnprotected();

    public abstract double getMinPower(double dt);
    public abstract double getMaxPower(double dt);
    public abstract double getMaxExtensionSpeed();
    public abstract double getMaxExtensionAcceleration();

    public void retract() {
        setSetpoint(0d);
        resetRunningSum();
    }

    public void resetRunningSum() {
        setRunningSum(0d);
    }

    public void addToRunningSum(double runningTerm) {
        setRunningSum(getRunningSum() + runningTerm);
    }

    public Gearbox getGearboxExtension() {
        return gearboxExtension;
    }

    public void setGearboxExtension(Gearbox gearboxExtension) {
        this.gearboxExtension = gearboxExtension;
    }

    public void setSetpoint(double setpoint) {
        if(setpoint != getSetpoint() && (getMotionProfile() == null || getMotionProfile().isDone())) {
            if(getExtensionControl().ordinal() == ExtensionControl.TRAPEZOIDAL.ordinal()) {
                setMotionProfile(new TrapezoidalMotionProfileGenerator(setpoint, getGearboxExtension().getPosition(), getGearboxExtension().getVelocity(), getMaxExtensionSpeed(), getMaxExtensionAcceleration()));
            } else if(getExtensionControl().ordinal() == ExtensionControl.RESIDUAL_VIBRATION_REDUCTION.ordinal()) {
                setMotionProfile(new ResidualVibrationReductionMotionProfilerGenerator(getGearboxExtension().getPosition(), setpoint - getGearboxExtension().getPosition(), getMaxExtensionSpeed(), getMaxExtensionAcceleration()));
            }

            updateSetpointUnprotected().accept(setpoint);
        }
    }

    public ExtensionControl getExtensionControl() {
        return extensionControl;
    }

    public void setExtensionControl(ExtensionControl extensionControl) {
        this.extensionControl = extensionControl;
    }

    public ControlConstants getControlConstants() {
        return controlConstants;
    }

    public void setControlConstants(ControlConstants controlConstants) {
        this.controlConstants = controlConstants;
    }

    public double getLastError() {
        return lastError;
    }

    public void setLastError(double lastError) {
        this.lastError = lastError;
    }

    public double getRunningSum() {
        return runningSum;
    }

    public void setRunningSum(double runningSum) {
        this.runningSum = runningSum;
    }

    public IMotionProfile getMotionProfile() {
        return motionProfile;
    }

    public void setMotionProfile(IMotionProfile motionProfile) {
        this.motionProfile = motionProfile;
    }
}
