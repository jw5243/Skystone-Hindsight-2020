package org.firstinspires.ftc.teamcode.ftc15026.auto.actions;

import org.firstinspires.ftc.teamcode.ftc15026.statemachines.CollectorExtensionStateMachine;
import org.firstinspires.ftc.teamcode.ftc15026.subsystems.Collector;

public class ExtendCollectorAction implements Action {
    private Collector collector;
    private double dt;
    private CollectorExtensionStateMachine.State extensionState;

    public ExtendCollectorAction(Collector collector, double dt, CollectorExtensionStateMachine.State extensionState) {
        setCollector(collector);
        setDt(dt);
        setExtensionState(extensionState);
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {
        getCollector().setSetpoint(getCollector().getSetpoint() + getDt() * Math.signum(getExtensionState().getPower()) * getCollector().getMaxExtensionSpeed());
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void done() {

    }

    public Collector getCollector() {
        return collector;
    }

    public void setCollector(Collector collector) {
        this.collector = collector;
    }

    public double getDt() {
        return dt;
    }

    public void setDt(double dt) {
        this.dt = dt;
    }

    public CollectorExtensionStateMachine.State getExtensionState() {
        return extensionState;
    }

    public void setExtensionState(CollectorExtensionStateMachine.State extensionState) {
        this.extensionState = extensionState;
    }
}
