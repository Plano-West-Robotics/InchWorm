package org.firstinspires.ftc.teamcode.inchworm.units;

public class Distance {
    /** 0 distance in any unit. Use for convenience, like Angle.ZERO */
    public static final Distance ZERO = Distance.inches(0);

    // val in inches (🇺🇸 lol)
    private final double val;

    // conversion factors
    private static final double INCHES_PER_FOOT = 12;
    private static final double INCHES_PER_TILE = 23.625;
    private static final double CM_PER_INCH = 2.54;

    /**
     * Encoder ticks per motor revolution for your drive motors. You can find this information online.
     */
    public static final double TICKS_PER_REV = 560;
    /**
     * Diameter of your mecanum wheels in inches.
     */
    public static final Distance WHEEL_DIAMETER = Distance.inches(3);
    /**
     * Encoder ticks per inch rotated
     */
    public static final double TPI = TICKS_PER_REV / (WHEEL_DIAMETER.distInInches() * Math.PI);

    private Distance(double v) {
        val = v;
    }

    /**
     * Create a distance from inches
     * @param val Distance in inches
     * @return Distance object containing the distance
     */
    public static Distance inches(double val) {
        return new Distance(val);
    }

    /**
     * Create a distance from feet
     * @param val Distance in feet
     * @return Distance object containing the distance
     */
    public static Distance feet(double val) {
        return new Distance(val * INCHES_PER_FOOT);
    }

    /**
     * Create a distance from FTC field tiles.
     * Note this means the distance from the center of one tile to the center of another, not the side length of one actual tile.
     * @param val Distance in FTC field tiles
     * @return Distance object containing the distance.
     */
    public static Distance tiles(double val) {
        return new Distance(val * INCHES_PER_TILE);
    }

    /**
     * Distance moved after rotating for a certain number of encoder ticks.
     * Don't use this, this is mainly for internal InchWorm usage.
     */
    public static Distance ticks(double val) {
        return new Distance(val / TPI);
    }

    /**
     * Create a distance from centimeters
     * @param val Distance in centimeters
     * @return Distance object containing the distance.
     */
    public static Distance centimeters(double val) {
        return new Distance(val / CM_PER_INCH);
    }

    /**
     * Returns the distance in inches
     * @return distance in inches
     */
    public double distInInches() {
        return val;
    }

    /**
     * Returns the distance in feet
     * @return distance in feet
     */
    public double distInFeet() {
        return val / INCHES_PER_FOOT;
    }

    /**
     * Returns the distance in FTC field tiles
     * Note this means the distance from the center of one tile to the center of another, not the side length of one actual tile.
     * @return distance in FTC field tiles
     */
    public double distInTiles() {
        return val / INCHES_PER_TILE;
    }

    /**
     * Returns the number of times the encoders ticked by traveling this far.
     * Only for internal use.
     */
    public double distInTicks() {
        return val * TPI;
    }

    /**
     * Returns the distance in centimeters
     * @return Distance in centimeters
     */
    public double distInCentimeters() {
        return val * CM_PER_INCH;
    }

    /**
     * Returns the sum of two distances.
     * @param a first distance
     * @param b second distance
     * @return a new Distance object containing the sum of the two distances.
     */
    public static Distance add(Distance a, Distance b) {
        return Distance.inches(a.distInInches() + b.distInInches());
    }

    /**
     * Returns the difference of two distances.
     * @param a first distance
     * @param b second distance
     * @return a new Distance object containing the sum of the two distances.
     */
    public static Distance sub(Distance a, Distance b) {
        return Distance.inches(a.distInInches() - b.distInInches());
    }
}
