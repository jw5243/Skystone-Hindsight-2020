package org.firstinspires.ftc.teamcode.ftc15026;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ftc15026.statemachines.CollectorExtensionStateMachine;
import org.firstinspires.ftc.teamcode.ftc15026.statemachines.SpindleStateMachine;

@TeleOp(name = "Main Teleop")
public abstract class Main extends SkystoneRobot {
    @Override
    public void init() {
        super.init();
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        super.loop();
        //Update the collector extension based on the sign of the right joystick in the x-direction
        //for the second controller user.
        CollectorExtensionStateMachine.updateState(
                gamepad2.right_stick_x > 0 ? CollectorExtensionStateMachine.State.EXTEND :
                        gamepad2.right_stick_x < 0 ? CollectorExtensionStateMachine.State.RETRACT :
                                CollectorExtensionStateMachine.State.IDLE
        );
        
        if(gamepad1.y || gamepad2.y) {
            SpindleStateMachine.updateState(SpindleStateMachine.State.IDLE);
        } else if(gamepad1.x || gamepad2.x) {
            SpindleStateMachine.updateState(SpindleStateMachine.State.OUTTAKE);
        } else if(CollectorExtensionStateMachine.getState().ordinal() !=
                CollectorExtensionStateMachine.State.IDLE.ordinal()) {
            SpindleStateMachine.updateState(SpindleStateMachine.State.INTAKE);
        }
    }

    @Override
    public void stop() {
        super.stop();
    }
}
