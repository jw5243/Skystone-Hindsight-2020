package org.firstinspires.ftc.teamcode.ftc15026.auto.actions;

import org.firstinspires.ftc.teamcode.lib.util.Time;
import org.firstinspires.ftc.teamcode.lib.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;
import org.firstinspires.ftc.teamcode.lib.util.TimeUtil;

/**
 * Action to wait for a given amount of time to use this Action, call runAction(new WaitAction(your_time))
 */
public class WaitAction implements Action {
    private TimeProfiler timeProfiler;
    private double timeToWait;

    public WaitAction(Time timeToWait) {
        setTimeProfiler(new TimeProfiler(false));
        setTimeToWait(timeToWait.getTimeValue(TimeUnits.SECONDS));
    }

    @Override
    public void start() {
        getTimeProfiler().start();
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return getTimeProfiler().getDeltaTime(TimeUnits.SECONDS, false) >= getTimeToWait();
    }

    @Override
    public void done() {}

    public TimeProfiler getTimeProfiler() {
        return timeProfiler;
    }

    public void setTimeProfiler(TimeProfiler timeProfiler) {
        this.timeProfiler = timeProfiler;
    }

    public double getTimeToWait() {
        return timeToWait;
    }

    public void setTimeToWait(double timeToWait) {
        this.timeToWait = timeToWait;
    }
}
