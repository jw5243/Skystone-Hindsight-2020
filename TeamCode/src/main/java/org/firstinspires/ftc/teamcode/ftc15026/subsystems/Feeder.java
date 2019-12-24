package org.firstinspires.ftc.teamcode.ftc15026.subsystems;

import org.firstinspires.ftc.teamcode.lib.drivers.RevCRServo;
import org.firstinspires.ftc.teamcode.lib.drivers.RevServo;

public class Feeder extends Extension {
    private static Feeder instance;

    public static Feeder getInstance(Gearbox feederExtension, RevServo feederDumper, RevCRServo feederRotator) {
        if(instance == null) {
            instance = new Feeder(feederExtension, feederDumper, feederRotator);
        }

        return instance;
    }

    private RevServo feederDumper;
    private RevCRServo feederRotator;

    public Feeder(Gearbox feederExtension, RevServo feederDumper, RevCRServo feederRotator) {
        super(feederExtension);
        setFeederDumper(feederDumper);
        setFeederRotator(feederRotator);
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void update(double dt) {
        super.update(dt);
    }

    @Override
    public void stop() {
        super.stop();
    }

    public RevServo getFeederDumper() {
        return feederDumper;
    }

    public void setFeederDumper(RevServo feederDumper) {
        this.feederDumper = feederDumper;
    }

    public RevCRServo getFeederRotator() {
        return feederRotator;
    }

    public void setFeederRotator(RevCRServo feederRotator) {
        this.feederRotator = feederRotator;
    }
}
