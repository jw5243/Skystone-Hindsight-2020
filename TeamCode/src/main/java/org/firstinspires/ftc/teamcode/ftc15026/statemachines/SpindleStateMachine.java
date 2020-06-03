package org.firstinspires.ftc.teamcode.ftc15026.statemachines;

import org.firstinspires.ftc.teamcode.ftc15026.SkystoneRobot;

public class SpindleStateMachine {
    private static SkystoneRobot skystoneRobot;
    private static volatile State state = State.IDLE;
    private static State desiredState = State.IDLE;

    public enum State {
        IDLE(0d), INTAKE(1d), OUTTAKE(-0.8d);

        private final double power;

        State(double power) {
            this.power = power;
        }

        public double getPower() {
            return power;
        }
    }

    public static void init(SkystoneRobot skystoneRobot) {
        setSkystoneRobot(skystoneRobot);
    }

    public static void updateState(State desiredState) {
        //setState(desiredState);
        setDesiredState(desiredState);
    }

    public static void update() {
        if(!getState().equals(getDesiredState())) {
            if(getDesiredState().equals(State.OUTTAKE)) {
                getSkystoneRobot().getCollector().retract();
                if(FeederRotatorStateMachine.checkStateFinished(FeederRotatorStateMachine.State.STONE_TO_OUTTAKE) &&
                    FeederDumperStateMachine.hasReachedStateGoal()) {
                    setState(State.OUTTAKE);
                }
            } else {
                setState(getDesiredState());
            }
        }
    }

    public static boolean hasReachedStateGoal() {
        return getState().equals(getDesiredState());
    }

    public static State getState() {
        return state;
    }

    private static void setState(State state) {
        SpindleStateMachine.state = state;
    }

    public static SkystoneRobot getSkystoneRobot() {
        return skystoneRobot;
    }

    public static void setSkystoneRobot(SkystoneRobot skystoneRobot) {
        SpindleStateMachine.skystoneRobot = skystoneRobot;
    }

    public static State getDesiredState() {
        return desiredState;
    }

    public static void setDesiredState(State desiredState) {
        SpindleStateMachine.desiredState = desiredState;
    }
}
