package org.firstinspires.ftc.teamcode.tune;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.inchworm.InchWorm;

@Autonomous(group="tune")
public class SpeedTuner extends LinearOpMode {
    @Override
    public void runOpMode() {
        InchWorm inchWorm = new InchWorm(this,
                InchWorm.GLOBAL_ORIENTATION,
                InchWorm.POSE_ZERO);


        waitForStart();

        double lastY = 0;
        double veloSum = 0;
        int numMeasurements = 0;

        ElapsedTime timer = new ElapsedTime();
        double time = getRuntime() + 1;
        inchWorm.moveWheels(0, 1, 0, 1);
        while (getRuntime() < time) {
            double y = inchWorm.tracker.currentPos.y.distInTicks();

            double velo = (y - lastY) / timer.seconds();
            veloSum += velo;
            numMeasurements++;

            timer.reset();
            lastY = y;
            inchWorm.tracker.update();
        }

        inchWorm.moveWheels(0, 0, 0, 0);
        time = getRuntime() + 1;
        inchWorm.moveWheels(0, 0, 1, 1);

        double angSum = 0;
        int angMeasurements = 0;

        while (getRuntime() < time) {
            double current = Math.abs(inchWorm.imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate);
            angSum += current;
            angMeasurements++;
        }

        inchWorm.moveWheels(0, 0, 0, 0);
        while (opModeIsActive()) {
            telemetry.addData("max velocity", veloSum / numMeasurements);
            telemetry.addData("max angular velocity", angSum / angMeasurements);
            telemetry.update();
        }
    }
}
