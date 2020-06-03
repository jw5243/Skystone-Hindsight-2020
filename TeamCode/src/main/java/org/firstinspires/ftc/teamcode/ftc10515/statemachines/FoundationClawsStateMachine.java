package org.firstinspires.ftc.teamcode.ftc10515.statemachines;

public class FoundationClawsStateMachine {
    private static State state = State.INITIALIZATION;

    public enum State {
        INITIALIZATION(0.2d), GRAB_FOUNDATION(0.8d);

        private final double leftPosition;
        private final double rightPosition;

        State(final double leftPosition) {
            this.leftPosition = leftPosition;
            this.rightPosition = 1d - leftPosition;
        }

        public double getLeftPosition() {
            return leftPosition;
        }

        public double getRightPosition() {
            return rightPosition;
        }
    }

    public static void updateState(State desiredState) {
        setState(desiredState);
    }

    public static State getState() {
        return state;
    }

    public static void setState(State state) {
        FoundationClawsStateMachine.state = state;
    }
}
