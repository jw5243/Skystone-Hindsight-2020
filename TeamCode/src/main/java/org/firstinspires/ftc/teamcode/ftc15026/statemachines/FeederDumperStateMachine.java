package org.firstinspires.ftc.teamcode.ftc15026.statemachines;

import org.firstinspires.ftc.teamcode.ftc15026.SkystoneRobot;
import org.firstinspires.ftc.teamcode.lib.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;

public class FeederDumperStateMachine {
    private static final double STATE_CHANGE_TRANSITION_TIME = 0.5d; //s

    private static SkystoneRobot skystoneRobot;
    private static volatile State state = State.INITIALIZATION;
    private static TimeProfiler timeProfiler;

    public enum State {
        INITIALIZATION(0.5d), UNDER_SKYBRIDGE(0.5d),
        READY_FOR_LIFTOFF(0.65d), DUMP(0.31d), STONE_TO_OUTTAKE(0.55d);

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
        setTimeProfiler(new TimeProfiler(true));
    }

    public static void updateState(State desiredState) {
        if(desiredState.ordinal() != getState().ordinal()) {
            setState(desiredState);
            getTimeProfiler().getDeltaTime(TimeUnits.SECONDS, true);
        }
    }

    public static boolean hasReachedStateGoal() {
        return getTimeProfiler().getDeltaTime(TimeUnits.SECONDS, false) >= getStateChangeTransitionTime();
    }

    public static State getState() {
        return state;
    }

    private static void setState(State state) {
        FeederDumperStateMachine.state = state;
    }

    public static SkystoneRobot getSkystoneRobot() {
        return skystoneRobot;
    }

    public static void setSkystoneRobot(SkystoneRobot skystoneRobot) {
        FeederDumperStateMachine.skystoneRobot = skystoneRobot;
    }

    public static double getStateChangeTransitionTime() {
        return STATE_CHANGE_TRANSITION_TIME;
    }

    public static TimeProfiler getTimeProfiler() {
        return timeProfiler;
    }

    public static void setTimeProfiler(TimeProfiler timeProfiler) {
        FeederDumperStateMachine.timeProfiler = timeProfiler;
    }
}
