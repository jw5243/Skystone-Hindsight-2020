package org.firstinspires.ftc.teamcode.ftc15026.statemachines;

import org.firstinspires.ftc.teamcode.ftc15026.SkystoneRobot;

public class FeederRotatorStateMachine {
    private static final double ROTATOR_THRESHOLD = 0.1d;

    private static SkystoneRobot skystoneRobot;
    private static volatile State state = State.INITIALIZATION;
    private static State desiredState = State.INITIALIZATION;

    public enum State {
        INITIALIZATION(2.03d), UNDER_SKYBRIDGE(1.63d),
        READY_FOR_LIFTOFF(2.5d), STONE_TO_OUTTAKE(2.1d);

        private final double position;

        State(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
    }

    public static void init(SkystoneRobot skystoneRobot) {
        setSkystoneRobot(skystoneRobot);
    }

    public static void updateState(State desiredState) {
        setDesiredState(desiredState);
        if(desiredState.equals(State.UNDER_SKYBRIDGE)) {
            FeederStopperStateMachine.updateState(FeederStopperStateMachine.State.NOT_BLOCKING);
        }
    }

    public static void update() {
        if(!getState().equals(getDesiredState())) {
            if(getDesiredState().equals(State.UNDER_SKYBRIDGE)) {
                if(FeederStopperStateMachine.hasReachedStateGoal() && FeederStopperStateMachine.getState().equals(FeederStopperStateMachine.State.NOT_BLOCKING) &&
                    CollectorDumperStateMachine.getState().equals(CollectorDumperStateMachine.State.ON_GROUND) && CollectorDumperStateMachine.hasReachedStateGoal()) {
                    setState(getDesiredState());
                    FeederDumperStateMachine.updateState(FeederDumperStateMachine.State.UNDER_SKYBRIDGE);
                }
            } else {
                if(getDesiredState().equals(State.READY_FOR_LIFTOFF)) {
                    FeederStopperStateMachine.updateState(FeederStopperStateMachine.State.BLOCKING);
                    FeederDumperStateMachine.updateState(FeederDumperStateMachine.State.READY_FOR_LIFTOFF);
                } else if(getDesiredState().equals(State.STONE_TO_OUTTAKE)) {
                    FeederStopperStateMachine.updateState(FeederStopperStateMachine.State.NOT_BLOCKING);
                    FeederDumperStateMachine.updateState(FeederDumperStateMachine.State.STONE_TO_OUTTAKE);
                    SpindleStateMachine.updateState(SpindleStateMachine.State.OUTTAKE);
                }

                setState(getDesiredState());
            }
        }
    }

    public static boolean hasReachedStateGoal() {
        return getSkystoneRobot().getFeeder().getRotatorSetpoint() - getSkystoneRobot().getFeeder().getPotentiometer().getVoltage() <= getRotatorThreshold();
    }

    public static boolean checkStateFinished(State state) {
        return getState().equals(state) && hasReachedStateGoal() && getDesiredState().equals(getState());
    }

    public static State getState() {
        return state;
    }

    private static void setState(State state) {
        FeederRotatorStateMachine.state = state;
    }

    public static SkystoneRobot getSkystoneRobot() {
        return skystoneRobot;
    }

    public static void setSkystoneRobot(SkystoneRobot skystoneRobot) {
        FeederRotatorStateMachine.skystoneRobot = skystoneRobot;
    }

    public static double getRotatorThreshold() {
        return ROTATOR_THRESHOLD;
    }

    public static State getDesiredState() {
        return desiredState;
    }

    public static void setDesiredState(State desiredState) {
        FeederRotatorStateMachine.desiredState = desiredState;
    }
}
