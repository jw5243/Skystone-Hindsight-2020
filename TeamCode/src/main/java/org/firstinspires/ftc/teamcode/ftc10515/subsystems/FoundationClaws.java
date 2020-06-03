package org.firstinspires.ftc.teamcode.ftc10515.subsystems;

import org.firstinspires.ftc.teamcode.ftc10515.statemachines.FoundationClawsStateMachine;
import org.firstinspires.ftc.teamcode.lib.drivers.RevServo;

public class FoundationClaws implements Subsystem {
    private static FoundationClaws instance;

    public static FoundationClaws getInstance(RevServo leftClaw, RevServo rightClaw) {
        if(instance == null) {
            instance = new FoundationClaws(leftClaw, rightClaw);
        }

        return instance;
    }

    private RevServo leftClaw;
    private RevServo rightClaw;

    public FoundationClaws(RevServo leftClaw, RevServo rightClaw) {
        setLeftClaw(leftClaw);
        setRightClaw(rightClaw);
    }

    @Override
    public void start() {
        FoundationClawsStateMachine.updateState(FoundationClawsStateMachine.State.INITIALIZATION);
        getLeftClaw().setPosition(FoundationClawsStateMachine.getState().getLeftPosition());
        getRightClaw().setPosition(FoundationClawsStateMachine.getState().getRightPosition());
    }

    @Override
    public void update(double dt) {
        getLeftClaw().setPosition(FoundationClawsStateMachine.getState().getLeftPosition());
        getRightClaw().setPosition(FoundationClawsStateMachine.getState().getRightPosition());
    }

    @Override
    public void stop() {

    }

    public RevServo getLeftClaw() {
        return leftClaw;
    }

    public void setLeftClaw(RevServo leftClaw) {
        this.leftClaw = leftClaw;
    }

    public RevServo getRightClaw() {
        return rightClaw;
    }

    public void setRightClaw(RevServo rightClaw) {
        this.rightClaw = rightClaw;
    }
}
