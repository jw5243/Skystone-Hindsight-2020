package org.firstinspires.ftc.teamcode.ftc10515;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.ftc10515.statemachines.FlywheelStateMachine;
import org.firstinspires.ftc.teamcode.ftc10515.statemachines.FoundationClawsStateMachine;
import org.firstinspires.ftc.teamcode.ftc10515.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;

/**
 * This {@code class} acts as the driver-controlled program for FTC team 10515 for the Skystone
 * challenge. By extending {@code SkystoneRobot}, we already have access to all the robot subsystems,
 * so only tele-operated controls need to be defined here.
 *
 * @see SkystoneRobot
 * @see Subsystem
 */
@TeleOp(name = "Main Teleop 10515")
@Disabled
@Deprecated
public class Main extends SkystoneRobot {
    private Rotation2d targetHeading;

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        super.loop();
        //Control the drivetrain based on user input of the joysticks.
        setDrivetrainPower(new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, new Rotation2d(gamepad1.right_stick_x, false)));

        if(gamepad1.left_trigger > 0.05 || gamepad2.left_trigger > 0.05) {
            FlywheelStateMachine.updateState(FlywheelStateMachine.State.INTAKE);
        } else if(gamepad1.right_trigger > 0.05 || gamepad2.right_trigger > 0.05) {
            FlywheelStateMachine.updateState(FlywheelStateMachine.State.OUTTAKE);
        } else if(gamepad1.y || gamepad2.y) {
            FlywheelStateMachine.updateState(FlywheelStateMachine.State.IDLE);
        }

        if(getEnhancedGamepad1().isDpadUpJustPressed()) {
            getFeeder().autoExtend();
            //FeederExtensionStateMachine.updateState(FeederExtensionStateMachine.State.EXTEND);
        } else if(getEnhancedGamepad1().isDpadDownJustPressed()) {
            getFeeder().fullyRetract();
            //FeederExtensionStateMachine.updateState(FeederExtensionStateMachine.State.RETRACT);
        } else {
            //FeederExtensionStateMachine.updateState(FeederExtensionStateMachine.State.IDLE);
        }

        if(gamepad1.left_bumper) {
            FoundationClawsStateMachine.updateState(FoundationClawsStateMachine.State.GRAB_FOUNDATION);
        } else if(gamepad1.right_bumper) {
            FoundationClawsStateMachine.updateState(FoundationClawsStateMachine.State.INITIALIZATION);
        }

        //This implements basic operation of the number of stones stacked for automated stacking.
        if(getEnhancedGamepad2().isDpadUpJustPressed() || getEnhancedGamepad1().isDpadRightJustPressed()) {
            getStackTracker().addStoneToStack();
        } else if(getEnhancedGamepad2().isDpadDownJustPressed() || getEnhancedGamepad1().isDpadLeftJustPressed()) {
            getStackTracker().resetStack();
        }

        //Give a setpoint heading for the robot to stay oriented towards.
        if(getEnhancedGamepad1().isLeftStickButtonJustPressed() || getEnhancedGamepad1().isRightStickButtonJustPressed()) {
            setTargetHeading(getRobotStateEstimator().getHeading());
        }

        //Restrict user motion to only move in the direction that is pointed towards the most, so
        //user error in moving straight or strafing is omitted.
        if(getTargetHeading() != null && getEnhancedGamepad1().isLeft_stick_button() || getEnhancedGamepad1().isRight_stick_button()) {
            //Supply full turning power when off by 30 degrees or more
            final double kP = 0d;//1 / Math.toRadians(30d);
            //Get the error in the heading of the robot
            double error = getTargetHeading().getRadians() - getRobotStateEstimator().getHeading().getRadians();
            //Calculate the feedback power that the robot needs to supply to compensate for the error.
            double feedback = -kP * error;

            //Only allow the robot to move either forward or strafe
            //TODO: Use drive encoders to ensure that the robot only moves straight or sideways
            if(Math.abs(gamepad1.left_stick_y) > Math.abs(gamepad1.left_stick_x)) {
                setDrivetrainPower(new Pose2d(-gamepad1.left_stick_y, 0d, new Rotation2d(feedback, false)));
            } else {
                setDrivetrainPower(new Pose2d(0d, -gamepad1.left_stick_x, new Rotation2d(feedback, false)));
            }
        }

        telemetry.addLine("Robot position: " + getRobotStateEstimator().getPose());
        telemetry.addLine("Stones stacked: " + getStackTracker());
        telemetry.addLine("Feeder power (right): " + getFeeder().getGearboxExtension().getPower());
        telemetry.addLine("Feeder power (left) :" + getFeeder().getGearboxExtension().getMotors()[1].getLastPower());
        telemetry.addLine("Feeder position: " + getFeeder().getGearboxExtension().getPosition());
        telemetry.addLine("Feeder setpoint: " + getFeeder().getSetpoint());
    }

    public Rotation2d getTargetHeading() {
        return targetHeading;
    }

    public void setTargetHeading(Rotation2d targetHeading) {
        this.targetHeading = targetHeading;
    }
}
