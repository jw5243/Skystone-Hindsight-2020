package org.firstinspires.ftc.teamcode.ftc15026.subsystems;

public class Superstructure implements Subsystem {
    private static Superstructure instance;

    public static Superstructure getInstance() {
        if(instance == null) {
            instance = new Superstructure();
        }

        return instance;
    }

    public Superstructure() {

    }

    @Override
    public void start() {

    }

    @Override
    public void update(double dt) {

    }

    @Override
    public void stop() {

    }
}
