package org.firstinspires.ftc.teamcode.ftc15026.auto.actions;

import org.firstinspires.ftc.teamcode.ftc15026.statemachines.CollectorDumperStateMachine;
import org.firstinspires.ftc.teamcode.ftc15026.subsystems.Collector;

public class FlipCollectorDumperAction implements Action {
    private CollectorDumperStateMachine.State state;

    public FlipCollectorDumperAction(CollectorDumperStateMachine.State state) {
        setState(state);
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {
        CollectorDumperStateMachine.updateState(getState());
        Collector.getInstance().getCollectorDumper().setPosition(CollectorDumperStateMachine.getState().getPosition());
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void done() {

    }

    public CollectorDumperStateMachine.State getState() {
        return state;
    }

    public void setState(CollectorDumperStateMachine.State state) {
        this.state = state;
    }
}
