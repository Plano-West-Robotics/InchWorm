package org.firstinspires.ftc.teamcode.inchworm.units;

public class Angle {
    public static final Angle ZERO = new Angle(0);
    // value in radians
    private final double val;

    private Angle(double v) {
        val = v;
    }

    public static Angle radians(double ang) {
        return new Angle(ang);
    }

    public static Angle degrees(double ang) {
        return new Angle(Math.toRadians(ang));
    }

    public double angleInRadians() {
        return val;
    }

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

        // convert back to radians when done
        return Angle.radians(angle);
    }

    /**
     * Returns the real (smallest) difference between two angles.
     * @param a first angle (in radians)
     * @param b second angle (in radians)
     * @return smallest difference between the two angles, within range [-π, π)
     */
    public static Angle sub(Angle a, Angle b) {
        double diff = a.angleInRadians() - b.angleInRadians();
        if (diff >= Math.PI) diff -= 2 * Math.PI;
        if (diff < -Math.PI) diff += 2 * Math.PI;
        return Angle.radians(diff);
    }

    public static Angle add(Angle a, Angle b) {
        return Angle.radians(a.angleInRadians() + b.angleInRadians());
    }

    public Angle neg() {
        return Angle.radians(-val);
    }
}
