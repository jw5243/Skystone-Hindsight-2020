package org.firstinspires.ftc.robotcontroller.internal;

import org.opencv.core.Point3;

public class VisionLibrary {
    public native static void filterSkystone(long addrRgba);
    public native static void filterStone(long addrRgba);
    public native static Point3[] skystoneContour(long addrRgba);
    public native static Point3[] stoneContour(long addrRgba);
    public native static void stoneDetector(long addrRgba);
    public native static void skystoneDetector(long addrRgba);
}
