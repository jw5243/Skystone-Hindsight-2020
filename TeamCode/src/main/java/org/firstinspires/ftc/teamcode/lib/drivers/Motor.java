package org.firstinspires.ftc.teamcode.lib.drivers;

public enum Motor {
    GOBILDA_312_RPM(312, 537.6),
    GOBILDA_223_RPM(223, 753.2),
    NEVERST_3_7(1780, 103.6);

    private final double RPM;
    private final double ENCODER_TICKS_PER_REVOLUTION;

    Motor(double RPM, double encoderTicksPerRevolution) {
        this.RPM = RPM;
        this.ENCODER_TICKS_PER_REVOLUTION = encoderTicksPerRevolution;
    }

    public double maxAngularVelocity() {
        return getRPM() * Math.PI / 30d;
    }

    public double getRPM() {
        return RPM;
    }

    public double getENCODER_TICKS_PER_REVOLUTION() {
        return ENCODER_TICKS_PER_REVOLUTION;
    }
}
