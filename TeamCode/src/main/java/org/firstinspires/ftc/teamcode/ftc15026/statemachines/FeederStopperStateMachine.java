package org.firstinspires.ftc.teamcode.ftc15026.statemachines;

import org.firstinspires.ftc.teamcode.ftc15026.SkystoneRobot;
import org.firstinspires.ftc.teamcode.lib.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;

public class FeederStopperStateMachine {
    private static final double STATE_CHANGE_TRANSITION_TIME = 0.5d; //s

    private static SkystoneRobot skystoneRobot;
    private static volatile State state = State.INITIALIZATION;
    private static State desiredState = State.BLOCKING;
    private static TimeProfiler timeProfiler;

    private static boolean countingForLiftoff = false;

    public enum State {
        INITIALIZATION(0.45d), NOT_BLOCKING(0.45d), BLOCKING(0.28d);

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
        setDesiredState(desiredState);
    }

    public static void update() {
        //If we wish to block the feeder to prevent it from falling into the robot,
        //we must check other feeder states.
        if(!getState().equals(getDesiredState())) {
            if(getDesiredState().equals(State.BLOCKING) && FeederRotatorStateMachine.checkStateFinished(FeederRotatorStateMachine.State.READY_FOR_LIFTOFF)) {
                if(!isCountingForLiftoff()) {
                    setCountingForLiftoff(true);
                    getTimeProfiler().getDeltaTime(true);
                } else if(getTimeProfiler().getDeltaTime(TimeUnits.SECONDS, false) > 2d) {
                    setCountingForLiftoff(false);
                    setState(State.BLOCKING);
                    getTimeProfiler().getDeltaTime(TimeUnits.SECONDS, true);
                }
            } else if(getDesiredState().equals(State.NOT_BLOCKING)) {
                setState(State.NOT_BLOCKING);
                getTimeProfiler().getDeltaTime(TimeUnits.SECONDS, true);
            }
        }
    }

    public static boolean hasReachedStateGoal() {
        return getTimeProfiler().getDeltaTime(TimeUnits.SECONDS, false) > getStateChangeTransitionTime();
    }

    public static State getState() {
        return state;
    }

    private static void setState(State state) {
        FeederStopperStateMachine.state = state;
    }

    public static SkystoneRobot getSkystoneRobot() {
        return skystoneRobot;
    }

    public static void setSkystoneRobot(SkystoneRobot skystoneRobot) {
        FeederStopperStateMachine.skystoneRobot = skystoneRobot;
    }

    public static double getStateChangeTransitionTime() {
        return STATE_CHANGE_TRANSITION_TIME;
    }

    public static TimeProfiler getTimeProfiler() {
        return timeProfiler;
    }

    public static void setTimeProfiler(TimeProfiler timeProfiler) {
        FeederStopperStateMachine.timeProfiler = timeProfiler;
    }

    public static State getDesiredState() {
        return desiredState;
    }

    public static void setDesiredState(State desiredState) {
        FeederStopperStateMachine.desiredState = desiredState;
    }

    public static boolean isCountingForLiftoff() {
        return countingForLiftoff;
    }

    public static void setCountingForLiftoff(boolean countingForLiftoff) {
        FeederStopperStateMachine.countingForLiftoff = countingForLiftoff;
    }
}
