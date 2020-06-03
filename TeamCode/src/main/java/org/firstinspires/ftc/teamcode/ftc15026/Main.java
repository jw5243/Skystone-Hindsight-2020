package org.firstinspires.ftc.teamcode.ftc15026;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.ftc15026.statemachines.CollectorDumperStateMachine;
import org.firstinspires.ftc.teamcode.ftc15026.statemachines.CollectorExtensionStateMachine;
import org.firstinspires.ftc.teamcode.ftc15026.statemachines.FeederDumperStateMachine;
import org.firstinspires.ftc.teamcode.ftc15026.statemachines.FeederExtensionStateMachine;
import org.firstinspires.ftc.teamcode.ftc15026.statemachines.FeederRotatorStateMachine;
import org.firstinspires.ftc.teamcode.ftc15026.statemachines.FeederStopperStateMachine;
import org.firstinspires.ftc.teamcode.ftc15026.statemachines.SpindleStateMachine;
import org.firstinspires.ftc.teamcode.ftc15026.subsystems.Collector;
import org.firstinspires.ftc.teamcode.ftc15026.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.lib.vision.SkystoneNavigation;

/**
 * This {@code class} acts as the driver-controlled program for FTC team 15026 for the Skystone
 * challenge. By extending {@code SkystoneRobot}, we already have access to all the robot subsystems,
 * so only tele-operated controls need to be defined here.
 *
 * The controls for this robot are:
 *  User 1:
 *      Drive:
 *          Left & Right joysticks -> Mecanum drive
 *          Joystick buttons       -> Snap-to-grid driving
 *          Left-trigger           -> Auto foundation movement
 *      Collector:
 *          Dpad-left              -> Collector setpoint such that dumper is still on the ground, but
 *                                    retracted as much as possible. This allows for safer movement while
 *                                    being permitted to cross under the skybridge with the outtake rotated
 *                                    into the robot.
 *          Dpad-right             -> Collector retracts fully into the robot, making it easy for the user
 *                                    to transition from intake to outtake.
 *          Y-button (pressed)     -> Stop spindle from running.
 *          X-button (pressed)     -> Sets spindle state to outtake stone.
 *      //Vision:
 *          Left bumper (pressed)  -> Auto feed
 *          Right bumper (pressed) -> Auto align to collect stones
 *  User 2:
 *      Drive:
 *          Joystick buttons       -> Snap-to-grid driving
 *      Collector:
 *          Right joystick (x)     -> Extend and retract collector (right -> positive, left -> negative).
 *          Y-button (pressed)     -> Stop spindle from running.
 *          X-button (pressed)     -> Sets spindle state to outtake stone.
 *      Feeder:
 *          Dpad-up                -> Extend feeder to stacked height based on counter class controlled
 *                                    by the second user.
 *          Dpad-down              -> Fully retracts feeder.
 *          Dpad-right             -> Adds stone to stack tracker (NOTE: only changes when feeder is retracted)
 *          Dpad-left              -> Removes stone from stack tracker (NOTE: only changes when feeder is retracted)
 *          Left-trigger           -> Resets stack tracker (NOTE: only changes when feeder is retracted)
 *          Left bumper (pressed)  -> Toggles whether the feeder is rotated into the robot or ready to extend.
 *      //Vision:
 *          Right bumper (pressed) -> Toggle between camera views (front or back camera)
 *
 * @see SkystoneRobot
 * @see Subsystem
 */
@TeleOp(name = "Main Teleop 15026")
public class Main extends SkystoneRobot {
    private Rotation2d targetHeading;
    private volatile boolean isAutomating;
    private volatile SkystoneNavigation skystoneNavigation;

    @Override
    public void init() {
        super.init();
        isAutomating = false;
        skystoneNavigation = new SkystoneNavigation();
        new Thread(() -> {
            skystoneNavigation.init(hardwareMap, telemetry, VuforiaLocalizer.CameraDirection.BACK, true);
            while(true) {
                skystoneNavigation.loop();
                try {
                    Thread.sleep(20);
                } catch(InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }).start();

        msStuckDetectLoop = 30000;
    }

    @Override
    public void start() {
        super.start();
        FeederRotatorStateMachine.updateState(FeederRotatorStateMachine.State.READY_FOR_LIFTOFF);
    }

    @Override
    public void loop() {
        super.loop();
        //skystoneNavigation.loop();
        //Control the drivetrain based on user input of the joysticks.
        if(!isAutomating) {
            setDrivetrainPower(new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, new Rotation2d(gamepad1.right_stick_x, false)));
            updateCollector();
            updateFeeder();
            snapToGridDriveMode();
            if(gamepad1.left_trigger > 0.05d) {
                autoMoveFoundation();
            } else if(getEnhancedGamepad1().isLeftBumperJustPressed()) {
                alignWithSkystone();
            }
        }

        getDrive().setDesiredPose(new Pose2d(0d, 5d, new Rotation2d(0d, false)));

        telemetry.addLine("Is automating: " + isAutomating);
        telemetry.addLine("Skystone Location: " + skystoneNavigation.queueSkystonLocation());
        telemetry.addLine("Robot Location: " + getRobotStateEstimator().getPose());
        telemetry.addLine("Robot Power: " + getDrivetrainPower());
        //telemetry.addLine("Root Mean Squared Error: " + getDrive().getRmsFoundation());
        telemetry.addLine("Stones stacked: " + getStackTracker());
        telemetry.addLine("Stacked Height: " + getStackTracker().getExtensionHeight());
        //telemetry.addLine("Collector Power: " + getCollector().getGearboxExtension().getPower());
        //telemetry.addLine("Collector Position: " + getCollector().getGearboxExtension().getPosition());
        //telemetry.addLine("Collector Setpoint: " + getCollector().getSetpoint());
        //telemetry.addLine("Collector State: " + CollectorExtensionStateMachine.getState());
        telemetry.addLine("Feeder Power: " + getFeeder().getGearboxExtension().getPower());
        telemetry.addLine("Feeder Position: " + getFeeder().getGearboxExtension().getPosition());
        telemetry.addLine("Feeder Setpoint: " + getFeeder().getSetpoint());
        //telemetry.addLine("Feeder Rotator State: " + FeederRotatorStateMachine.getState());
        //telemetry.addLine("Feeder Rotator Desired State: " + FeederRotatorStateMachine.getDesiredState());
        telemetry.addLine("Feeder Rotator Setpoint: " + getFeeder().getRotatorSetpoint());
        telemetry.addLine("Feeder Potentiometer: " + getFeeder().getPotentiometer().getVoltage());
        telemetry.addLine("Feeder Angle: " + getFeeder().getRotatedAngle().getDegrees());
        //telemetry.addLine("Feeder Stopper State: " + FeederStopperStateMachine.getState());
        //telemetry.addLine("Feeder Stopper Desired State: " + FeederStopperStateMachine.getDesiredState());
        //telemetry.addLine("Feeder Finished: " + FeederRotatorStateMachine.hasReachedStateGoal());
        telemetry.addLine("Ready for liftoff: " + FeederRotatorStateMachine.checkStateFinished(FeederRotatorStateMachine.State.READY_FOR_LIFTOFF));
    }

    @Override
    public void stop() {
        super.stop();
        if(skystoneNavigation != null) {
            skystoneNavigation.stop();
        }
    }

    public void updateCollector() {
        //Update the collector extension based on the sign of the right joystick in the x-direction
        //for the second controller user.
        if(!SpindleStateMachine.getDesiredState().equals(SpindleStateMachine.State.OUTTAKE)) {
            CollectorExtensionStateMachine.updateState(
                    gamepad2.right_stick_x > 0 ? CollectorExtensionStateMachine.State.EXTEND :
                            gamepad2.right_stick_x < 0 ? CollectorExtensionStateMachine.State.RETRACT :
                                    CollectorExtensionStateMachine.State.IDLE
            );
        }

        //Remove access to the collector unless the feeder is for certain not going to get in the way
        //of the intake.
        if(FeederRotatorStateMachine.getState().equals(FeederRotatorStateMachine.State.READY_FOR_LIFTOFF) ||
            FeederRotatorStateMachine.getState().equals(FeederRotatorStateMachine.State.STONE_TO_OUTTAKE)) {
            if(getEnhancedGamepad1().isDpadLeftJustPressed()) {
                CollectorExtensionStateMachine.updateState(CollectorExtensionStateMachine.State.EXTEND);
                getCollector().setSetpoint(Collector.getCollectorDumperRotateThreshold());
            } else if(getEnhancedGamepad1().isDpadRightJustPressed()) {
                CollectorExtensionStateMachine.updateState(CollectorExtensionStateMachine.State.RETRACT);
                getCollector().retract();
            }

            //This portion of the code automates the intake process of the spindle, but makes the other
            //controls for the spindle manually operated by either user.
            if(gamepad1.y || gamepad2.y) {
                SpindleStateMachine.updateState(SpindleStateMachine.State.IDLE);
            } else if((gamepad1.x || gamepad2.x) && !CollectorExtensionStateMachine.getState().equals(
                    CollectorExtensionStateMachine.State.EXTEND)) {
                //SpindleStateMachine.updateState(SpindleStateMachine.State.OUTTAKE);
                FeederRotatorStateMachine.updateState(FeederRotatorStateMachine.State.STONE_TO_OUTTAKE);
            } else if(!CollectorExtensionStateMachine.getState().equals(CollectorExtensionStateMachine.State.IDLE) &&
                    !SpindleStateMachine.getDesiredState().equals(SpindleStateMachine.State.OUTTAKE)) {
                SpindleStateMachine.updateState(SpindleStateMachine.State.INTAKE);
            }
        }
    }

    public void updateFeeder() {
        //This implements basic operation of the number of stones stacked for automated stacking.
        if(FeederStopperStateMachine.getState().equals(FeederStopperStateMachine.State.BLOCKING) &&
                FeederStopperStateMachine.hasReachedStateGoal()) {
            if(getEnhancedGamepad2().isDpadUpJustPressed()) {
                FeederDumperStateMachine.updateState(
                        FeederDumperStateMachine.getState().equals(FeederDumperStateMachine.State.READY_FOR_LIFTOFF) ?
                                FeederDumperStateMachine.State.DUMP : FeederDumperStateMachine.State.READY_FOR_LIFTOFF);
                //FeederExtensionStateMachine.updateState(FeederExtensionStateMachine.State.EXTEND);
                //getFeeder().resetRunningSum();
            } else if(getEnhancedGamepad2().isDpadDownJustPressed()) {
                FeederExtensionStateMachine.updateState(FeederExtensionStateMachine.State.RETRACT);
                //getStackTracker().resetStack();
                getFeeder().resetRunningSum();
            }
        }

        //Allow the user to change the stack size.
        if(!FeederExtensionStateMachine.getState().equals(FeederExtensionStateMachine.State.EXTEND)) {
            if(getEnhancedGamepad2().isDpadRightJustPressed()) {
                getStackTracker().addStoneToStack();
            } else if(getEnhancedGamepad2().isDpadLeftJustPressed()) {
                getStackTracker().removeStoneFromStack();
            } else if(gamepad2.left_trigger > 0.05) {
                getStackTracker().resetStack();
            }

            if(getEnhancedGamepad2().isLeftBumperJustPressed()) {
                FeederRotatorStateMachine.updateState(FeederRotatorStateMachine.getState().equals(
                        FeederRotatorStateMachine.State.READY_FOR_LIFTOFF) ? FeederRotatorStateMachine.State.UNDER_SKYBRIDGE :
                        FeederRotatorStateMachine.State.READY_FOR_LIFTOFF);
                if(FeederRotatorStateMachine.getDesiredState().equals(FeederRotatorStateMachine.State.READY_FOR_LIFTOFF)) {
                    getFeeder().setSetpoint(0d);
                }
            }
        }
    }

    public void snapToGridDriveMode() {
        //Give a setpoint heading for the robot to stay oriented towards.
        if(getEnhancedGamepad1().isLeftStickButtonJustPressed() || getEnhancedGamepad1().isRightStickButtonJustPressed()) {
            setTargetHeading(getRobotStateEstimator().getHeading());
        }

        //Restrict user motion to only move in the direction that is pointed towards the most, so
        //user error in moving straight or strafing is omitted.
        if(getTargetHeading() != null && getEnhancedGamepad1().isLeft_stick_button() || getEnhancedGamepad1().isRight_stick_button()) {
            //Supply full turning power when off by 30 degrees or more
            final double kP = 1 / Math.toRadians(30d);
            //Get the error in the heading of the robot
            double error = getTargetHeading().getRadians() - getRobotStateEstimator().getHeading().getRadians();
            //Calculate the feedback power that the robot needs to supply to compensate for the error.
            double feedback = kP * error;

            //Only allow the robot to move either forward or strafe
            //TODO: Use drive encoders to ensure that the robot only moves straight or sideways
            if(Math.abs(gamepad1.left_stick_y) > Math.abs(gamepad1.left_stick_x)) {
                setDrivetrainPower(new Pose2d(-gamepad1.left_stick_y, 0d, new Rotation2d(feedback, false)));
            } else {
                setDrivetrainPower(new Pose2d(0d, -gamepad1.left_stick_x, new Rotation2d(feedback, false)));
            }
        }
    }

    public void alignWithSkystone() {
        //Pose2d skystoneLocation = skystoneNavigation.queueSkystonLocation().inverse();
        //if(skystoneLocation != null) {
        //    isAutomating = true;
            //SimpleMatrix skystoneLocationRelativeToRobot = new SimpleMatrix(6, 1, false, new double[]{
            //    skystoneLocation.getTranslation().y(), 0d, -skystoneLocation.getTranslation().x(), 0d, skystoneLocation.getRotation().getRadians(), 0d
            //});

            //TODO: Run MPC algorithm based on skystone location
            //TODO: Override drive motors to follow MPC trajectory

            //new Thread(getFeeder().autoStack(() -> isAutomating = !isAutomating)).start();

        //    getDrive().driveToLocation(this::toggleAutomation, skystoneLocation);
        //}

        if(skystoneNavigation.queueSkystonLocation() != null) {
            isAutomating = true;
            new Thread(getDrive().driveToLocation(this::toggleAutomation, gamepad1, gamepad2)).start();
        }
    }

    public void autoMoveFoundation() {
        isAutomating = true;
        new Thread(getDrive().autoMoveFooundation(this::toggleAutomation)).start();
    }

    public void toggleAutomation() {
        isAutomating = !isAutomating;
    }

    public Rotation2d getTargetHeading() {
        return targetHeading;
    }

    public void setTargetHeading(Rotation2d targetHeading) {
        this.targetHeading = targetHeading;
    }
}
