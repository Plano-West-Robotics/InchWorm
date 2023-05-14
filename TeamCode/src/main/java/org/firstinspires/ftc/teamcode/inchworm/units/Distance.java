package org.firstinspires.ftc.teamcode.inchworm.units;

public class Distance {
    public static final Distance ZERO = Distance.inches(0);

    // val in inches (🇺🇸 lol)
    private final double val;

    private static final double INCHES_PER_FOOT = 12;
    private static final double INCHES_PER_TILE = 23.625;
    /**
     * Encoder ticks per motor revolution for your drive motors. You can find this information online.
     */
    public static final double TICKS_PER_REV = 560;
    /**
     * Diameter of your mecanum wheels in inches.
     */
    public static final double WHEEL_DIAMETER_INCHES = 3;
    /**
     * Encoder ticks per inch rotated
     */
    public static final double TPI = TICKS_PER_REV / (WHEEL_DIAMETER_INCHES * Math.PI);

    private Distance(double v) {
        val = v;
    }

    public static Distance inches(double val) {
        return new Distance(val);
    }

    public static Distance feet(double val) {
        return new Distance(val * INCHES_PER_FOOT);
    }

    public static Distance tiles(double val) {
        return new Distance(val * INCHES_PER_TILE);
    }

    public static Distance ticks(double val) {
        return new Distance(val / TPI);
    }

    public double distInInches() {
        return val;
    }

    public double distInFeet() {
        return val / INCHES_PER_FOOT;
    }

    public double distInTiles() {
        return val / INCHES_PER_TILE;
    }

    public double distInTicks() {
        return val * TPI;
    }

    public static Distance add(Distance a, Distance b) {
        return Distance.inches(a.distInInches() + b.distInInches());
    }

    public static Distance sub(Distance a, Distance b) {
        return Distance.inches(a.distInInches() - b.distInInches());
    }
}
