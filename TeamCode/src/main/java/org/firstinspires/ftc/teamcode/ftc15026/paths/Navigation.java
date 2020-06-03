package org.firstinspires.ftc.teamcode.ftc15026.paths;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ftc15026.statemachines.FeederRotatorStateMachine;

@Autonomous
public class Navigation extends PathFollowingSkystoneRobot {
    @Override
    protected Runnable autonomousSequence() {
        return () -> FeederRotatorStateMachine.updateState(FeederRotatorStateMachine.State.READY_FOR_LIFTOFF);
    }
}
