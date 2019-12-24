package org.firstinspires.ftc.teamcode.ftc15026.subsystems;

import org.firstinspires.ftc.teamcode.ftc15026.statemachines.CollectorDumperStateMachine;
import org.firstinspires.ftc.teamcode.ftc15026.statemachines.CollectorExtensionStateMachine;
import org.firstinspires.ftc.teamcode.ftc15026.statemachines.SpindleStateMachine;
import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;
import org.firstinspires.ftc.teamcode.lib.drivers.RevServo;

public class Collector extends Extension {
    private static final double COLLECTOR_DUMPER_ROTATE_THRESHOLD = 5d;

    private static Collector instance;

    public static Collector getInstance(RevMotor collectorExtension, RevMotor spindle, RevServo collectorDumper) {
        if(instance == null) {
            instance = new Collector(collectorExtension, spindle, collectorDumper);
        }

        return instance;
    }

    public static Collector getInstance() {
        return instance;
    }

    private RevMotor spindle;
    private RevServo collectorDumper;

    public Collector(RevMotor collectorExtension, RevMotor spindle, RevServo collectorDumper) {
        super(collectorExtension);
        setSpindle(spindle);
        setCollectorDumper(collectorDumper);
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void update(double dt) {
        super.update(dt);
        //Set collector dumper position in the robot or on the ground based on the drawer slide extension position.
        //Only do this if the intake is currently trying to extend or retract, to ensure that if the
        //drawer slides extend due to the drivetrain's momentum, the container will not unnecessarily
        //flip up and down.
        if(CollectorExtensionStateMachine.getState().ordinal() != CollectorExtensionStateMachine.State.IDLE.ordinal()) {
            CollectorDumperStateMachine.updateState(
                    getGearboxExtension().getPosition() > getCollectorDumperRotateThreshold() ?
                            CollectorDumperStateMachine.State.ON_GROUND : CollectorDumperStateMachine.State.IN_ROBOT);
        }

        getCollectorDumper().setPosition(CollectorDumperStateMachine.getState().getPosition());
        getSpindle().setPower(SpindleStateMachine.getState().getPower());
    }

    @Override
    public void stop() {
        super.stop();
        CollectorExtensionStateMachine.updateState(CollectorExtensionStateMachine.State.IDLE);
        SpindleStateMachine.updateState(SpindleStateMachine.State.IDLE);
        getGearboxExtension().setPower(CollectorExtensionStateMachine.getState().getPower());
        getSpindle().setPower(SpindleStateMachine.getState().getPower());
    }

    public RevMotor getSpindle() {
        return spindle;
    }

    public void setSpindle(RevMotor spindle) {
        this.spindle = spindle;
    }

    public RevServo getCollectorDumper() {
        return collectorDumper;
    }

    public void setCollectorDumper(RevServo collectorDumper) {
        this.collectorDumper = collectorDumper;
    }

    public static double getCollectorDumperRotateThreshold() {
        return COLLECTOR_DUMPER_ROTATE_THRESHOLD;
    }
}
