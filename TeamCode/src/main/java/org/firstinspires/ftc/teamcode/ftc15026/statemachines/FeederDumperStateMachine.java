package org.firstinspires.ftc.teamcode.ftc15026.statemachines;

public class FeederDumperStateMachine {
    private static State state = State.INITIALIZATION;

    public enum State {
        INITIALIZATION(0d), UNDER_SKYBRIDGE(0d), READY_FOR_LIFTOFF(0d), DUMP(0d);

        private final double position;

        State(double position) {
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
        FeederDumperStateMachine.state = state;
    }
}
