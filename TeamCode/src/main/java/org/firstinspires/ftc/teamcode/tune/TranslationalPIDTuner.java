package org.firstinspires.ftc.teamcode.tune;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.inchworm.WormUtil;
import org.firstinspires.ftc.teamcode.inchworm.InchWorm;
import org.firstinspires.ftc.teamcode.inchworm.PIDController;
import org.firstinspires.ftc.teamcode.inchworm.units.Angle;
import org.firstinspires.ftc.teamcode.inchworm.units.Distance;

@Config
@Autonomous(group="tune")
public class TranslationalPIDTuner extends LinearOpMode {
    public static final double MAX_VEL = 2000;
    public static double TARGET = Distance.tiles(1).distInTicks();
    public static double Kp = 0;
    public static double Ki = 0;
    public static double Kd = 0;
    /*
     * This class should be used to tune translational PID for InchWorm.
     */
    @Override
    public void runOpMode() {
        WormUtil wormUtil = new WormUtil(this, InchWorm.GLOBAL_ORIENTATION);
        InchWorm inchWorm = new InchWorm(this,
                InchWorm.GLOBAL_ORIENTATION,
                InchWorm.POSE_ZERO);

        PIDController controller = new PIDController(Kp, Ki, Kd, TARGET);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        wormUtil.waitForStart();

        while (opModeIsActive()) {
            controller.setParams(Kp, Ki, Kd, TARGET);

            InchWorm.Pose current = inchWorm.tracker.currentPos;
            double out = controller.calculate(current.y.distInTicks());
            out /= MAX_VEL;

            if (gamepad1.x) {
                out = 0;
                controller.reset();
            }

            telemetry.addData("out", out);
            telemetry.addData("error", String.format("%.2f", TARGET - current.y.distInTicks()));
            telemetry.addData("current", String.format("%.2f", current.y.distInTicks()));
            telemetry.addData("target", String.format("%.2f", TARGET));
            telemetry.update();
            inchWorm.moveWheels(0, out, 0, 1);
            inchWorm.tracker.update();
        }
    }
}
