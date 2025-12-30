package org.firstinspires.ftc.teamcode.Limelight;

import java.util.TreeMap;
import java.util.Map;

public class LookupTableTest {
    private static final double TARGET_HEIGHT_IN = 38.75;
    private static final double LL_LENS_HEIGHT_IN = 14.0; //NEEDS TO BE ADJUSTED, JUST A PLACEHOLDER
    private static final double LL_MOUNT_ANGLE_DEG = 25.0; //NEEDS TO BE ADJUSTED, JUST A PLACEHOLDER

    private final TreeMap<Double, Double> rpmTable = new TreeMap<>();

    public LookupTableTest() {
        //RPM NUMBERS ARE JUST PLACEHOLDERS!! NEEDS TO BE CHANGED
        rpmTable.put(24.0, 1600.0);
        rpmTable.put(48.0, 2300.0);
        rpmTable.put(72.0, 2950.0);
        rpmTable.put(96.0, 3600.0);
    }

    public double calculateDistance(double ty) {
        double angleToGoalRad = Math.toRadians(LL_MOUNT_ANGLE_DEG + ty);
        // Formula: d = (h2 - h1) / tan(theta)
        return (TARGET_HEIGHT_IN - LL_LENS_HEIGHT_IN) / Math.tan(angleToGoalRad);
    }
}