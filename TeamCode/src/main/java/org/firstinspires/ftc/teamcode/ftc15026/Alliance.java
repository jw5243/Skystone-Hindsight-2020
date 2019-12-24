package org.firstinspires.ftc.teamcode.ftc15026;

public enum Alliance {
    BLUE, RED;

    public Alliance getOpposingAlliance() {
        return ordinal() == BLUE.ordinal() ? RED : BLUE;
    }

    public static Alliance getOpposingAlliance(Alliance alliance) {
        return alliance.ordinal() == BLUE.ordinal() ? RED : BLUE;
    }

    @Override
    public String toString() {
        return ordinal() == BLUE.ordinal() ? "Blue" : "Red";
    }
}
