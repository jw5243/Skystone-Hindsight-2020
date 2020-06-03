package org.firstinspires.ftc.teamcode.ftc10515.control;

public class StackTracker {
    private static final double STONE_HEIGHT = 4d;
    private static final double FOUNDATION_HEIHGT = 1d;
    private static final double FEEDER_DUMPER_HEIGHT = 2d;

    private static StackTracker instance;

    public static StackTracker getInstance() {
        if(instance == null) {
            instance = new StackTracker();
        }

        return instance;
    }

    private int stonesStacked;

    public StackTracker() {
        resetStack();
    }

    public double getExtensionHeight() {
        return getFoundationHeihgt() - getFeederDumperHeight() + (getStonesStacked() + 1) * getStoneHeight();
    }

    public void resetStack() {
        setStonesStacked(0);
    }

    public void addStoneToStack() {
        setStonesStacked(getStonesStacked() + 1);
    }

    @Override
    public String toString() {
        return "Stones stacked: " + getStonesStacked();
    }

    public static double getStoneHeight() {
        return STONE_HEIGHT;
    }

    public static double getFoundationHeihgt() {
        return FOUNDATION_HEIHGT;
    }

    public static double getFeederDumperHeight() {
        return FEEDER_DUMPER_HEIGHT;
    }

    public int getStonesStacked() {
        return stonesStacked;
    }

    public void setStonesStacked(int stonesStacked) {
        this.stonesStacked = stonesStacked;
    }
}
