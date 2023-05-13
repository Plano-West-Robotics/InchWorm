package org.firstinspires.ftc.teamcode.inchworm;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class WormUtil {
    LinearOpMode opMode;
    IMU imu;

    public WormUtil(LinearOpMode opMode, ImuOrientationOnRobot imuOrientationOnRobot) {
        this.opMode = opMode;
        imu = opMode.hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(imuOrientationOnRobot));

        /*
            TODO: this shouldn't really be needed i think, but it doesn't hurt.
         */
        imu.resetYaw();
    }

    public void pause(double seconds) {
        double time = opMode.getRuntime() + seconds;
        while (opMode.getRuntime() < time && !opMode.isStopRequested()) {}
    }

    public int getLargest(int x, int y, int z) {
        return Math.max(z, Math.max(x, y));
    }

    public void print(String s) {
        opMode.telemetry.addLine(s);
    }

    public void print(String caption, String value) {
        opMode.telemetry.addData(caption, value);
    }

    /**
     * Re-interprets current heading (AKA yaw) to be 0.
     * Make sure to call this function after turning.
     * @see WormUtil#getHeading()
     */
    public void reset() {
        imu.resetYaw();
    }

    /**
     * Get the current rotation around the Z-Axis (known as heading or yaw) since the last time reset() was called.
     * This value will be normalized to be within [-180, 180) <b>degrees, not radians</b>. It follows the right-hand-rule:
     * Positive values are <b>counter-clockwise</b> around the axis, negative values are <b>clockwise</b>.
     * @see WormUtil#reset()
     * @return the rotation since the last time reset() was called
     */
    public double getHeading() {
        return getHeading(AngleUnit.DEGREES);
    }

    /**
     * Get the current rotation around the Z-Axis (known as heading or yaw) since the last time reset() was called.
     * You can provide an AngleUnit to get the result in either degrees or radians
     * This value will be normalized to be within [-180, 180) degrees (or [-π, π) radians). It follows the right-hand-rule:
     * Positive values are <b>counter-clockwise</b> around the axis, negative values are <b>clockwise</b>.
     * @see WormUtil#reset()
     * @param angleUnit The unit for the result to be in
     * @return the rotation since the last time reset() was called, in `angleUnit`s.
     */
    public double getHeading(AngleUnit angleUnit) {
        return imu.getRobotYawPitchRollAngles().getYaw(angleUnit);
    }

    /**
     * LinearOpMode's waitForStart, but doesn't crash the robot when the stop button is pressed.
     */
    public void waitForStart() {
        while (!opMode.isStarted() && !opMode.isStopRequested());
    }
}
