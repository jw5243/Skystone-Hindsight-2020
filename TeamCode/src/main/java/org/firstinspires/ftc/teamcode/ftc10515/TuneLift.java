package org.firstinspires.ftc.teamcode.ftc10515;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Disabled
public class TuneLift extends SkystoneRobot {
    public static double kP = 0.06;
    public static double kI = 0.03;
    public static double kD = 0.005;

    public Constants constants = Constants.kP;

    public enum Constants {
        kP, kI, kD;

        public Constants next() {
            if(ordinal() == kP.ordinal()) {
                return kI;
            } else if(ordinal() == kI.ordinal()) {
                return kD;
            }

            return kP;
        }

        @Override
        public String toString() {
            return ordinal() == kP.ordinal() ? "kP" : ordinal() == kI.ordinal() ? "kI" : "kD";
        }
    }

    @Override
    public void loop() {
        super.loop();
        if(getEnhancedGamepad1().isDpadUpJustPressed()) {
            getFeeder().autoExtend();
            //FeederExtensionStateMachine.updateState(FeederExtensionStateMachine.State.EXTEND);
        } else if(getEnhancedGamepad1().isDpadDownJustPressed()) {
            getFeeder().fullyRetract();
            //FeederExtensionStateMachine.updateState(FeederExtensionStateMachine.State.RETRACT);
        }

        if(getEnhancedGamepad2().isDpadUpJustPressed() || getEnhancedGamepad1().isDpadRightJustPressed()) {
            getStackTracker().addStoneToStack();
        } else if(getEnhancedGamepad2().isDpadDownJustPressed() || getEnhancedGamepad1().isDpadLeftJustPressed()) {
            getStackTracker().resetStack();
        }

        if(getEnhancedGamepad1().isLeftBumperJustPressed()) {
            constants = constants.next();
        }

        if(getEnhancedGamepad1().isyJustPressed()) {
            if(constants.ordinal() == Constants.kP.ordinal()) {
                kP += 0.001d;
            } else if(constants.ordinal() == Constants.kI.ordinal()) {
                kI += 0.001d;
            } else {
                kD += 0.001d;
            }
        } else if(getEnhancedGamepad1().isxJustPressed()) {
            if(constants.ordinal() == Constants.kP.ordinal()) {
                kP -= 0.001d;
            } else if(constants.ordinal() == Constants.kI.ordinal()) {
                kI -= 0.001d;
            } else {
                kD -= 0.001d;
            }
        }

        telemetry.addLine("State: " + constants);
        telemetry.addLine("kP: " + kP);
        telemetry.addLine("kI: " + kI);
        telemetry.addLine("kD: " + kD);
        telemetry.addLine("Stones stacked: " + getStackTracker());
        telemetry.addLine("Feeder power (right): " + getFeeder().getGearboxExtension().getPower());
        telemetry.addLine("Feeder power (left) :" + getFeeder().getGearboxExtension().getMotors()[1].getLastPower());
        telemetry.addLine("Feeder position: " + getFeeder().getGearboxExtension().getPosition());
        telemetry.addLine("Feeder setpoint: " + getFeeder().getSetpoint());
    }
}
