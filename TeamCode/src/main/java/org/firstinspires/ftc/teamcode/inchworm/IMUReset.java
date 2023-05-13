package org.firstinspires.ftc.teamcode.inchworm;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Reset IMU", group = "util")
public class IMUReset extends LinearOpMode {
    @Override
    public void runOpMode() {
        WormUtil wormUtil = new WormUtil(this, InchWorm.GLOBAL_ORIENTATION);

        wormUtil.waitForStart();

        wormUtil.reset();
    }
}
