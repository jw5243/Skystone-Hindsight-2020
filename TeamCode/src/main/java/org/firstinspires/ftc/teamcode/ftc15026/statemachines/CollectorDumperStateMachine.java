package org.firstinspires.ftc.teamcode.ftc15026.statemachines;

public class CollectorDumperStateMachine {
    private static State state = State.INITIALIZATION;

    public enum State {
        INITIALIZATION(0.2d), IN_ROBOT(0.1d), ON_GROUND(0.7d);

        private final double position;

        State(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
    }

    public static void updateState(State desiredState) {
        //TODO: Check for superstructure
        setState(desiredState);
    }

    public static State getState() {
        return state;
    }

    public static void setState(State state) {
        CollectorDumperStateMachine.state = state;
    }
}
