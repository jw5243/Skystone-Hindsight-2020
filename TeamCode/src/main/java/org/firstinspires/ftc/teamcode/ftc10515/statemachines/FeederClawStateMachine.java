package org.firstinspires.ftc.teamcode.ftc10515.statemachines;

public class FeederClawStateMachine {
    private static State state = State.WAIT_FOR_STONE;

    public enum State {
        WAIT_FOR_STONE(0d), GRAB_STONE(0d), DROP_STONE(0d);

        private final double position;

        State(final double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
    }

    public static void updateState(State desiredState) {
        setState(desiredState);
    }

    public static State getState() {
        return state;
    }

    public static void setState(State state) {
        FeederClawStateMachine.state = state;
    }
}
