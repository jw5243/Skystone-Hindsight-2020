package org.firstinspires.ftc.teamcode.ftc15026.statemachines;

import org.firstinspires.ftc.teamcode.ftc15026.SkystoneRobot;

public class CollectorExtensionStateMachine {
    private static SkystoneRobot skystoneRobot;
    private static volatile State state = State.IDLE;

    public enum State {
        IDLE(0d), EXTEND(0.4d), RETRACT(-0.4d);

        private final double power;

        State(double power) {
            this.power = power;
        }

        public double getPower() {
            return power;
        }

        @Override
        public String toString() {
            return equals(IDLE) ? "Idle" : equals(EXTEND) ? "Extend" : "Retract";
        }
    }

    public static void init(SkystoneRobot skystoneRobot) {
        setSkystoneRobot(skystoneRobot);
    }

    public static void updateState(State desiredState) {
        setState(desiredState);
    }

    public static boolean hasReachedStateGoal() {
        return getSkystoneRobot().getCollector().hasReachedDesiredExtensionLength(1 / 4d);
    }

    public static State getState() {
        return state;
    }

    private static void setState(State state) {
        CollectorExtensionStateMachine.state = state;
    }

    public static SkystoneRobot getSkystoneRobot() {
        return skystoneRobot;
    }

    public static void setSkystoneRobot(SkystoneRobot skystoneRobot) {
        CollectorExtensionStateMachine.skystoneRobot = skystoneRobot;
    }
}
