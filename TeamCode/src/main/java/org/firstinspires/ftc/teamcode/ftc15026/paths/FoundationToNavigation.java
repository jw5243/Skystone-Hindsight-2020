package org.firstinspires.ftc.teamcode.ftc15026.paths;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ftc15026.statemachines.FeederRotatorStateMachine;
import org.firstinspires.ftc.teamcode.lib.control.AdaptivePurePursuitController;
import org.firstinspires.ftc.teamcode.lib.control.CurvePoint;

@Autonomous
public class FoundationToNavigation extends PathFollowingSkystoneRobot {
    private static final CurvePoint INIT_TO_FOUNDATION = new CurvePoint(
            0d, 24d, Math.toRadians(180d), 10f
    );

    @Override
    protected Runnable autonomousSequence() {
        return () -> {
            setController(new AdaptivePurePursuitController(INIT_TO_FOUNDATION));
            getStackTracker().resetStack();
            getStackTracker().addStoneToStack();
            FeederRotatorStateMachine.updateState(FeederRotatorStateMachine.State.READY_FOR_LIFTOFF);
            while(!FeederRotatorStateMachine.checkStateFinished(FeederRotatorStateMachine.State.READY_FOR_LIFTOFF)) {
                try {
                    Thread.sleep(40);
                } catch(InterruptedException e) {
                    e.printStackTrace();
                }
            }

            getFeeder().extend();
            while(!getController().hasReachedGoal()) {
                setDrivetrainPower(getController().follow(
                        this::getRobotPose, this::getRobotSpeed
                ));

                try {
                    Thread.sleep(40);
                } catch(InterruptedException e) {
                    e.printStackTrace();
                }
            }

            getFeeder().retract();
        };
    }
}
