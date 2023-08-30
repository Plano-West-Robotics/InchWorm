package org.firstinspires.ftc.teamcode.inchworm.units;

public class Angle {
    /** 0 degrees/radians, use for convenience */
    public static final Angle ZERO = new Angle(0);
    // value in radians
    private final double val;

    private Angle(double v) {
        val = v;
    }

    /**
     * Create an angle from radians
     * @param ang Angle in radians
     * @return Angle object containing the given angle
     */
    public static Angle radians(double ang) {
        return new Angle(ang);
    }

    /**
     * Create an angle from degrees
     * @param ang Angle in degrees
     * @return Angle object containing the given angle
     */
    public static Angle degrees(double ang) {
        return new Angle(Math.toRadians(ang));
    }

    /**
     * Returns this angle in radians
     * @return Angle in radians
     */
    public double angleInRadians() {
        return val;
    }

    /**
     * Returns this angle in degrees
     * @return Angle in degrees
     */
    public double angleInDegrees() {
        return Math.toDegrees(val);
    }

    /**
     * normalizes theta into [0, 2π)
     * @param theta angle to normalize in radians
     * @return theta normalized into [0, 2π)
     */
    public static Angle modAngle(Angle theta) {
        // convert to degrees because mod 2pi doesn't work?
        double angle = theta.angleInDegrees();

        angle += 360;
        angle %= 360;

        // convert back when done
        return Angle.degrees(angle);
    }

    /**
     * Returns the real (smallest) difference between two angles.
     * @param a first angle
     * @param b second angle
     * @return smallest difference between the two angles, within range [-π, π)
     */
    public static Angle sub(Angle a, Angle b) {
        double diff = a.angleInRadians() - b.angleInRadians();
        if (diff >= Math.PI) diff -= 2 * Math.PI;
        if (diff < -Math.PI) diff += 2 * Math.PI;
        return Angle.radians(diff);
    }

    /**
     * Returns the sum of two angles
     * @param a first angle
     * @param b second angle
     * @return sum of the two angles
     */
    public static Angle add(Angle a, Angle b) {
        return Angle.radians(a.angleInRadians() + b.angleInRadians());
    }

    /**
     * Returns the negative of this angle.
     * @return This angle multiplied by -1
     */
    public Angle neg() {
        return Angle.radians(-val);
    }
}
