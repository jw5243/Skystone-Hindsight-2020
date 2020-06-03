package org.firstinspires.ftc.teamcode.ftc15026.paths;

import org.firstinspires.ftc.teamcode.ftc15026.SkystoneRobot;
import org.firstinspires.ftc.teamcode.lib.control.AdaptivePurePursuitController;

public abstract class PathFollowingSkystoneRobot extends SkystoneRobot {
    private volatile AdaptivePurePursuitController controller;

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void start() {
        super.start();
        new Thread(autonomousSequence()).start();
    }

    @Override
    public void loop() {
        super.loop();
        telemetry.addLine("Robot Pose: " + getRobotStateEstimator().getPose());
    }

    protected abstract Runnable autonomousSequence();

    public AdaptivePurePursuitController getController() {
        return controller;
    }

    public void setController(AdaptivePurePursuitController controller) {
        this.controller = controller;
    }
}
