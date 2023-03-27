package org.firstinspires.ftc.teamcode.inchworm;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(group = "test")
public class InchTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        WormUtil wormUtil = new WormUtil(this);
        InchWorm inchWorm = new InchWorm(this);

        wormUtil.waitForStart();

        inchWorm.moveTo(24, 0);
        inchWorm.moveTo(24, 24);
        inchWorm.moveTo(24, 0);
        inchWorm.moveTo(0, 0, Math.PI / 2);

        inchWorm.moveTo(0, 0, 0);
    }
}
