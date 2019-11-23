package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class ServoTest extends LinearOpMode {

    Servo autoClutch = null;
    public void runOpMode() throws InterruptedException {
        autoClutch = hardwareMap.servo.get("autoClutch");

        waitForStart();

        autoClutch.setPosition(0);

        sleep(5000);
    }
}
