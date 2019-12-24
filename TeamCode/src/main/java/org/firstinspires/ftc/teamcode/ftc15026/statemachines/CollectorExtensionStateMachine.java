package org.firstinspires.ftc.teamcode.ftc15026.statemachines;

public class CollectorExtensionStateMachine {
    private static State state = State.IDLE;

    public enum State {
        IDLE(0d), EXTEND(0.4d), RETRACT(-0.4d);

        private final double power;

        State(double power) {
            this.power = power;
        }

        public double getPower() {
            return power;
        }
    }

    public static void updateState(State desiredState) {
        setState(desiredState);
    }

    public static State getState() {
        return state;
    }

    public static void setState(State state) {
        CollectorExtensionStateMachine.state = state;
    }
}
