package org.firstinspires.ftc.teamcode.ftc10515.auto.actions;

import org.firstinspires.ftc.teamcode.lib.util.Time;
import org.firstinspires.ftc.teamcode.lib.util.TimeUtil;

/**
 * Action to wait for a given amount of time To use this Action, call runAction(new WaitAction(your_time))
 */
public class WaitAction implements Action {
    private final Time mTimeToWait;
    private Time mStartTime;

    public WaitAction(Time timeToWait) {
        mTimeToWait = timeToWait;
    }

    @Override
    public void start() {
        mStartTime = TimeUtil.getCurrentRuntime();
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return TimeUtil.getCurrentRuntime().subtract(mStartTime).compareTo(mTimeToWait) >= 0;
    }

    @Override
    public void done() {}
}
