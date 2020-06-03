package org.firstinspires.ftc.teamcode.ftc10515.subsystems;

import org.firstinspires.ftc.teamcode.ftc10515.TuneLift;
import org.firstinspires.ftc.teamcode.ftc10515.control.StackTracker;
import org.firstinspires.ftc.teamcode.ftc10515.statemachines.FeederClawStateMachine;
import org.firstinspires.ftc.teamcode.lib.annotations.DisableFeederClaw;
import org.firstinspires.ftc.teamcode.lib.annotations.ExtensionBounds;
import org.firstinspires.ftc.teamcode.lib.annotations.MaxExtensionAcceleration;
import org.firstinspires.ftc.teamcode.lib.annotations.MaxExtensionSpeed;
import org.firstinspires.ftc.teamcode.lib.control.ControlConstants;
import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;
import org.firstinspires.ftc.teamcode.lib.drivers.RevServo;

@MaxExtensionSpeed(extensionSpeed = 5d)
@MaxExtensionAcceleration(extensionAcceleration = 10d)
@ExtensionBounds(extensionLength = 24d)
@DisableFeederClaw
public class Feeder extends Extension {
    private static Feeder instance;

    public static Feeder getInstance(RevMotor feederExtensionLeft, RevMotor feederExtensionRight,
                                     RevServo feederClaw) {
        if(instance == null) {
            instance = new Feeder(feederExtensionLeft, feederExtensionRight, feederClaw);
        }

        return instance;
    }

    public static Feeder getInstance() {
        return instance;
    }

    private RevServo feederClaw;

    public Feeder(RevMotor feederExtensionLeft, RevMotor feederExtensionRight,
                  RevServo feederClaw) {
        super(feederExtensionRight, feederExtensionLeft, ExtensionControl.TRAPEZOIDAL,
                new ControlConstants(0.06d, 0.03d, 0.01d/*0.12d, 0.25d, 0.05d*/, 0.13d, 0.01d, 0d
        ));

        setSetpoint(0d);
        setFeederClaw(feederClaw);
    }

    @Override
    public void start() {
        super.start();
        setSetpoint(0d);
        getGearboxExtension().setPower(0d);
    }

    @Override
    public void update(double dt) {
        super.update(dt);

        if(!isFeederClawDiabled()) {
            getFeederClaw().setPosition(FeederClawStateMachine.getState().getPosition());
        }
    }

    @Override
    public void stop() {
        super.stop();
    }

    /**
     * Based on the given height in {@code StackTracker}, the feeder extension drives up to that
     * height by setting the setpoint and generating a new motion profile based on the current
     * position of the lift. This allows for redirecting the lift quicker as it is not necessary to
     * wait for the lift to extend or retract the full way before generating a new profile.
     *
     * @see StackTracker
     * @see StackTracker#getExtensionHeight()
     * @see #generateMotionProfile()
     */
    public void autoExtend() {
        setSetpoint(StackTracker.getInstance().getExtensionHeight());
        resetRunningSum();
        //generateMotionProfile();
    }

    public RevServo getFeederClaw() {
        return feederClaw;
    }

    public void setFeederClaw(RevServo feederClaw) {
        this.feederClaw = feederClaw;
    }

    public boolean isFeederClawDiabled() {
        return getClass().isAnnotationPresent(DisableFeederClaw.class);
    }
}
