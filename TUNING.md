# The tuner's guide to `InchWorm`

First, open `Distance.java` in `inchworm/units` and fill out the required class constants, `TICKS_PER_REV` and `WHEEL_DIAMETER_INCHES`. You can find the ticks per revolution of your drive motors online.

Next, go into the main `InchWorm.java` file and edit GLOBAL_ORIENTATION to fit your robot. See [this page](https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html#orthogonal-mounting) for a guide to these values.

Then, run the `SpeedTuner` opMode. Let it run to the end, then check the telemetry on the driver station. These values are the maximum velocity and maximum angular velocity. Then place these values in the appropriate spots:
- Copy maximum velocity to the `MAX_VEL` field in `InchWorm.java` and `TranslationalPIDTuner.java`
- Copy maximum angular velocity to the `MAX_ANG_VEL` field in `InchWorm.java` and `TurnPIDTuner.java`

Now, you are ready to tune the PID controllers. Run `TranslationalPIDTuner` and open [the FTC dashboard](http://192.168.43.1:8080/dash) to see a graph of error over time. Use gamepad1's left and right bumpers to decrease and increase Kp by `scale`, and use dpad up and down to increase or decrease `scale` by 0.05. Once you are done tuning Kp, edit `TranslationalPIDTuner.java` to tune Ki and Kd instead. See [this page](https://www.ctrlaltftc.com/the-pid-controller/tuning-methods-of-a-pid-controller) for further detail on the PID tuning process. Once you are done tuning, copy your tuned Kp, Ki, and Kd values to their respective fields in `controllerX` and `controllerY` in `InchWorm.java`.

Repeat the above process for `TurnPIDTuner`, copying the tuned values to `controllerTheta` in `InchWorm.java`

Finally, you are done tuning. Run `InchTest` to ensure that you tuned everything correctly. Please feel free to reach out if you have any questions.
