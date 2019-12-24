package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class MecanumTest extends LinearOpMode {

    HardwareBot robot;
    public void runOpMode() throws InterruptedException {
        robot = new HardwareBot(this,true);

        waitForStart();

        while(opModeIsActive()) {
            robot.mecanum.TeleOpDriveMode();
        }
    }
}
