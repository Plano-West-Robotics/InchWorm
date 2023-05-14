package org.firstinspires.ftc.teamcode.inchworm;

import static org.firstinspires.ftc.teamcode.inchworm.units.Angle.ZERO;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.inchworm.units.Angle;
import org.firstinspires.ftc.teamcode.inchworm.units.Distance;

import java.util.List;

/**
 * InchWorm is a movement system aimed at making autonomous coding easy and effective.
 */
public class InchWorm {
    /** Default starting pose of (0, 0, 0) */
    public static final Pose POSE_ZERO = new Pose(Distance.ZERO, Distance.ZERO, Angle.ZERO);
    /** Global constant for hub orientation. Use this unless you need to override the orientation for one specific opmode.  */
    public static final RevHubOrientationOnRobot GLOBAL_ORIENTATION = new RevHubOrientationOnRobot(
            // TODO: change these if they are not accurate
            RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
            RevHubOrientationOnRobot.UsbFacingDirection.LEFT);


    private final DcMotor fl;
    private final DcMotor fr;
    private final DcMotor bl;
    private final DcMotor br;
    public final IMU imu;
    public final PositionTracker tracker = new PositionTracker();
    private int loopsCorrect = 0;
    /**
     * Maximum translational velocity in encoder ticks/second. Find this using the SpeedTuner
     */
    // TODO: tune this value
    private static final double MAX_VEL = 2000;
    /**
     * Maximum angular velocity in degrees/second. Find this using the SpeedTuner.
     */
    // TODO: tune this value
    private static final double MAX_ANG_VEL = -188;
    /**
     * PID controllers. <b>coefficients for controllerX and controllerY should be THE SAME!</b>
     * Tune controllerX and controllerY with the TranslationalPIDTuner, and tune controllerTheta with the TurnPIDTuner.
     */
    // TODO: tune these coefficients 
    private final PIDController controllerX = new PIDController(0, 0, 0, 0);
    private final PIDController controllerY = new PIDController(0, 0, 0, 0);
    private final PIDController controllerTheta = new PIDController(0, 0, 0, 0);

    private final LinearOpMode opMode;

    /**
     * speed multiplier, multiplied by the PID outputs.
     */
    private double speed = 1;

    private Pose target = tracker.currentPos;

    /**
     * Initialize an InchWorm object. Sets up motors, encoders, IMU, etc.
     * @param mode Current opMode. Used to print to telemetry and interface with hardware.
     * @param imuOrientationOnRobot Orientation of the IMU on the robot. In most cases, use GLOBAL_ORIENTATION.
     * @param startingPose Initial pose of the robot.
     */
    public InchWorm(LinearOpMode mode, ImuOrientationOnRobot imuOrientationOnRobot, Pose startingPose) {
        opMode = mode;
        HardwareMap hardwareMap = opMode.hardwareMap;

        fl = hardwareMap.get(DcMotor.class, "frontLeft");
        fr = hardwareMap.get(DcMotor.class, "frontRight");
        bl = hardwareMap.get(DcMotor.class, "rearLeft");
        br = hardwareMap.get(DcMotor.class, "rearRight");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(imuOrientationOnRobot));
        imu.resetYaw();

        // reset encoders to 0
        setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // this is a temporary measure. modes will be reset once actually moving
        setModes(DcMotor.RunMode.RUN_USING_ENCODER);

        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);

        // counteract inertia
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        tracker.setPoseEstimate(startingPose);
    }

    /**
     * Move to a specific position on the field.
     * @param pose Position to move to. For now, must be in inches and radians.
     */
    public void moveTo(Pose pose) {
        setTarget(pose);

        while (!update()) {}

        stop();
    }

    /** Set the target position
     * @param newTarget New target position
     */
    public void setTarget(Pose newTarget) {
        // convert pose in inches to pose in ticks & normalize angle to [-π, π] radians
        newTarget = newTarget.normalizeAngle();
        controllerX.setTarget(newTarget.x.distInTicks());
        controllerY.setTarget(newTarget.y.distInTicks());
        controllerTheta.setTarget(newTarget.theta.angleInDegrees());
        controllerX.reset();
        controllerY.reset();
        controllerTheta.reset();

        target = newTarget;
    }

    /**
     * Update wheel powers and return whether we have reached the target or not
     * @return Whether we have reached the target
     */
    public boolean update() {
        Pose current = tracker.currentPos.normalizeAngle();
        opMode.telemetry.addLine(current.toString());
        double angError = Angle.sub(target.theta, current.theta).angleInDegrees();
        opMode.telemetry.addData("angError", angError);
        double outX = controllerX.calculate(current.x.distInTicks());
        double outY = controllerY.calculate(current.y.distInTicks());
        double outTheta = controllerTheta.calculateWithError(angError);

        double a = current.theta.angleInRadians();
        double rotX = outX * Math.cos(a) - outY * Math.sin(a);
        double rotY = outX * Math.sin(a) + outY * Math.cos(a);
        outX /= MAX_VEL;
        outY /= MAX_VEL;
        outTheta /= MAX_ANG_VEL;
        opMode.telemetry.addLine("outX: " + outX + System.lineSeparator() + "outY: " + outY + System.lineSeparator() + "outTheta: " + outTheta);
        opMode.telemetry.update();

        double voltageCompensation = 12 / getBatteryVoltage();
        moveWheels(outX, outY, outTheta, getSpeedMultiplier() * voltageCompensation);
        tracker.update();

        return !isBusy(target, current);
    }

    private void setModes(DcMotor.RunMode mode) {
        fl.setMode(mode);
        fr.setMode(mode);
        bl.setMode(mode);
        br.setMode(mode);
    }

    /**
     * Whether motors are currently attempting to run to the target position.
     * Also includes a safeguard for stopping the opMode mid-move.
     * @return Whether all motors are busy, AND if the opMode is still running.
     */
    public boolean isBusy(Pose target, Pose current) {
        if (
                Math.abs(Distance.sub(target.x, current.x).distInTicks()) <= 20 &&
                Math.abs(Distance.sub(target.y , current.y).distInTicks()) <= 20 &&
                Math.abs(Angle.sub(target.theta, current.theta).angleInDegrees()) <= 5
        ) {
            loopsCorrect++;
        } else loopsCorrect = 0;

        return loopsCorrect <= 35 && opMode.opModeIsActive();
    }

    /**
     * Stops all motors
     */
    private void stop() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    /**
     * Move wheels based on simple mecanum mechanics. In most cases, you shouldn't need to use this.
     */
    public void moveWheels(double powerX, double powerY, double turn, double speed) {
        double flPower = (powerY - powerX + turn) * speed;
        double frPower = (powerY + powerX - turn) * speed;
        double blPower = (powerY + powerX + turn) * speed;
        double brPower = (powerY - powerX - turn) * speed;

        double scale = Math.max(1, (Math.abs(powerY) + Math.abs(turn) + Math.abs(powerX)) * Math.abs(speed)); // shortcut for max(abs([fl,fr,bl,br]))
        flPower /= scale;
        frPower /= scale;
        blPower /= scale;
        brPower /= scale;

        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);

    }


    /**
     * Get yaw from the IMU.
     * @param angleUnit Unit of the result.
     * @return yaw angle from the IMU, in `angleUnit` units.
     */
    public double getYaw(AngleUnit angleUnit) {
        return imu.getRobotYawPitchRollAngles().getYaw(angleUnit);
    }

    /**
     * Get yaw from the IMU in radians.
     * @return yaw angle from the IMU in radians.
     */
    public Angle getYaw() {
        return Angle.radians(getYaw(AngleUnit.RADIANS));
    }

    private double getBatteryVoltage() {
        // returns the root of mean of the squares of all the battery voltages
        double totalSquares = 0;
        int numSeen = 0;
        for (VoltageSensor sensor : opMode.hardwareMap.voltageSensor) {
            totalSquares += Math.pow(sensor.getVoltage(), 2);
            numSeen++;
        }

        return Math.sqrt(totalSquares / numSeen);
    }

    /**
     * Set the speed multiplier of the robot.
     * @param x new speed to set
     */
    public void setSpeedMultiplier(double x) {
        speed = x;
    }

    /**
     * Get the current speed multiplier.
     * @return current speed multiplier
     */
    public double getSpeedMultiplier() {
        return speed;
    }

    public static class Pose {
        public Distance x;
        public Distance y;
        public Angle theta = ZERO;

        public Pose(Distance X, Distance Y) {
            x = X;
            y = Y;
        }

        public Pose(Distance X, Distance Y, Angle angle) {
            x = X;
            y = Y;
            theta = angle;
        }

        /**
         * Normalizes theta into [0, 2π).
         * @see super.modAngle(double)
         * @return A new pose with theta normalized into [0, 2π).
         */
        public Pose normalizeAngle() {
            return new Pose(this.x, this.y, Angle.modAngle(this.theta));
        }

        /**
         * Rotates this pose by an angle.
         * @param angle angle to rotate by
         * @return new pose that is rotated by the angle
         */
        public Pose rot(Angle angle) {
            double a = angle.angleInRadians();
            double x = this.x.distInTicks();
            double y = this.y.distInTicks();
            double rotX = x * Math.cos(a) - y * Math.sin(a);
            double rotY = x * Math.sin(a) + y * Math.cos(a);

            return new Pose(Distance.ticks(rotX), Distance.ticks(rotY), this.theta);
        }

        /**
         * Add one pose to another.
         * @param other Other pose to add
         * @return A new pose that is the sum of `this` and `other`
         */
        public Pose add(Pose other) {
            return new Pose(Distance.add(this.x, other.x), Distance.add(this.y, other.y), Angle.add(this.theta, other.theta));
        }

        @NonNull
        public String toString() {
            return "x: " + x + System.lineSeparator() + "y: " + y + System.lineSeparator() + "theta: " + theta.angleInDegrees();
        }
    }

    public class PositionTracker {
        public Pose currentPos = POSE_ZERO;
        private int lastFL = 0;
        private int lastFR = 0;
        private int lastBL = 0;
        private int lastBR = 0;

        private int flOffset = 0;
        private int frOffset = 0;
        private int blOffset = 0;
        private int brOffset = 0;
        private Angle yawOffset = ZERO;

        /**
         * Set this to the diameter of the robot, in inches
         * Optional unless you need to relocalize with a custom angle.
         * Default: 0
         */
        private final Distance TRACKWIDTH = Distance.ZERO;

        /**
         * Override the current pose estimate.
         * @param pose Pose to relocalize to. x and y must be in inches.
         */
        public void setPoseEstimate(Pose pose) {
            pose = pose.normalizeAngle();
            double x = pose.x.distInTicks();
            double y = pose.y.distInTicks();
            double turn = (pose.theta.angleInRadians() * TRACKWIDTH.distInInches()) * Distance.TPI;

            int currentFL = fl.getCurrentPosition();
            int currentFR = fr.getCurrentPosition();
            int currentBL = bl.getCurrentPosition();
            int currentBR = br.getCurrentPosition();

            int fl = (int) (y - x + turn);
            int fr = (int) (y + x - turn);
            int bl = (int) (y + x + turn);
            int br = (int) (y - x - turn);


            lastFL = fl;
            lastFR = fr;
            lastBL = bl;
            lastBR = br;

            flOffset = fl - currentFL;
            frOffset = fr - currentFR;
            blOffset = bl - currentBL;
            brOffset = br - currentBR;
            yawOffset = Angle.sub(pose.theta, getYaw());

            currentPos = pose;
        }

        private double sinc(double x) {
            return x == 0 ? 1 : Math.sin(x) / x;
        }

        // this function doesn't really have a standard name, but it's similar to sinc so cosc it is
        // not to be confused with cosec
        private double cosc(double x) {
            return x == 0 ? 0 : (1 - Math.cos(x)) / x;
        }

        /**
         * Updates the current position estimate. The more you call this, the better.
         */
        public void update() {
            int newFL = fl.getCurrentPosition() + flOffset;
            int newFR = fr.getCurrentPosition() + frOffset;
            int newBL = bl.getCurrentPosition() + blOffset;
            int newBR = br.getCurrentPosition() + brOffset;

            Angle newYaw = Angle.add(getYaw(), yawOffset);
            Angle yawDiff = Angle.sub(newYaw, currentPos.theta);

            int flDiff = newFL - lastFL;
            int frDiff = newFR - lastFR;
            int blDiff = newBL - lastBL;
            int brDiff = newBR - lastBR;

            double yDiff = ((flDiff + frDiff + blDiff + brDiff) / 4.0);
            double xDiff = ((blDiff + frDiff - flDiff - brDiff) / 4.0);

            double expX = cosc(yawDiff.angleInRadians());
            double expY = sinc(yawDiff.angleInRadians());

            Pose posDiff = new Pose(Distance.ticks(yDiff * expX + xDiff * expY), Distance.ticks(yDiff * expY - xDiff * expX), yawDiff);
            posDiff = posDiff.rot(currentPos.theta.neg());

            currentPos = currentPos.add(posDiff);

            lastFL = newFL;
            lastFR = newFR;
            lastBL = newBL;
            lastBR = newBR;
        }
    }
}
