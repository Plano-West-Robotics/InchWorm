package org.firstinspires.ftc.teamcode.inchworm;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(group = "test")
public class InchTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        WormUtil wormUtil = new WormUtil(this, InchWorm.GLOBAL_ORIENTATION);
        InchWorm inchWorm = new InchWorm(this,
                InchWorm.GLOBAL_ORIENTATION,
                InchWorm.POSE_ZERO);

        wormUtil.waitForStart();

        inchWorm.moveTo(24, 0);
        inchWorm.moveTo(24, 24);
        inchWorm.moveTo(24, 0);
        inchWorm.moveTo(0, 0, Math.PI / 2);

        inchWorm.moveTo(0, 0, 0);
    }
}
