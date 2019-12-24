package org.firstinspires.ftc.teamcode.ftc15026.statemachines;

public class SpindleStateMachine {
    private static State state;

    public enum State {
        IDLE(0d), INTAKE(0.7d), OUTTAKE(-0.8d);

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
        SpindleStateMachine.state = state;
    }
}
