package org.firstinspires.ftc.teamcode.inchworm;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.inchworm.units.Angle;

@Autonomous(group = "test")
public class InchTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        WormUtil wormUtil = new WormUtil(this, InchWorm.GLOBAL_ORIENTATION);
        InchWorm inchWorm = new InchWorm(this,
                InchWorm.GLOBAL_ORIENTATION,
                InchWorm.POSE_ZERO);

        wormUtil.waitForStart();

        inchWorm.moveTo(new InchWorm.Pose(24, 0));
        inchWorm.moveTo(new InchWorm.Pose(24, 24));
        inchWorm.moveTo(new InchWorm.Pose(24, 0));
        inchWorm.moveTo(new InchWorm.Pose(0, 0, Angle.degrees(90)));

        inchWorm.moveTo(new InchWorm.Pose(0, 0, Angle.ZERO));
    }
}
