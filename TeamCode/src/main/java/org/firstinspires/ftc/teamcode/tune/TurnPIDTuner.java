package org.firstinspires.ftc.teamcode.tune;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.inchworm.WormUtil;
import org.firstinspires.ftc.teamcode.inchworm.InchWorm;
import org.firstinspires.ftc.teamcode.inchworm.PIDController;
import org.firstinspires.ftc.teamcode.inchworm.units.Angle;

@Config
@Autonomous(group="tune")
public class TurnPIDTuner extends LinearOpMode {
    public static final double MAX_ANG_VEL = -188;
    public static double TARGET = 90;
    public static double Kp = 5;
    public static double Ki = 0.15;
    public static double Kd = 0;
    /*
     * This class should be used to tune turn PID for InchWorm.
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

            double current = inchWorm.tracker.currentPos.theta.angleInDegrees();
            double out = controller.calculate(current);
            out /= MAX_ANG_VEL;

            if (gamepad1.x) {
                out = 0;
                controller.reset();
            }

            telemetry.addData("out", out);
            telemetry.addData("error", String.format("%.2f", TARGET - current));
            telemetry.addData("current", String.format("%.2f", current));
            telemetry.addData("target", String.format("%.2f", TARGET));
            telemetry.update();

            inchWorm.moveWheels(0, 0, out, 0.5);
            inchWorm.tracker.update();
        }
    }
}
