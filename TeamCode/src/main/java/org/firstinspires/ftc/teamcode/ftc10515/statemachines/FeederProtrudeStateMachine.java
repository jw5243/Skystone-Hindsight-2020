package org.firstinspires.ftc.teamcode.ftc10515.statemachines;

public class FeederProtrudeStateMachine {
    private static State state = State.IDLE;

    public enum State {
        IDLE(0d), EXTEND(0.2d), RETRACT(-0.2d);

        private final double power;

        State(final double power) {
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
        FeederProtrudeStateMachine.state = state;
    }
}
