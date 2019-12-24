package org.firstinspires.ftc.teamcode.ftc15026.subsystems;

import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;

public abstract class Extension implements Subsystem {
    private Gearbox gearboxExtension;
    private ExtensionControl extensionControl;
    private double setpoint;

    public enum ExtensionControl {
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
        FAST_RESIDUAL_VIBRATION_REDUCTION
    }

    public Extension(RevMotor motor) {
        this(new Gearbox(motor));
    }

    public Extension(RevMotor motor1, RevMotor motor2) {
        this(new Gearbox(motor1, motor2));
    }

    public Extension(Gearbox gearbox) {
        this(gearbox, ExtensionControl.BANG_BANG);
    }

    public Extension(Gearbox gearbox, ExtensionControl extensionControl) {
        setGearboxExtension(gearbox);
        setExtensionControl(extensionControl);
    }

    @Override
    public void start() {

    }

    @Override
    public void update(double dt) {

    }

    @Override
    public void stop() {

    }

    public Gearbox getGearboxExtension() {
        return gearboxExtension;
    }

    public void setGearboxExtension(Gearbox gearboxExtension) {
        this.gearboxExtension = gearboxExtension;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public ExtensionControl getExtensionControl() {
        return extensionControl;
    }

    public void setExtensionControl(ExtensionControl extensionControl) {
        this.extensionControl = extensionControl;
    }
}
