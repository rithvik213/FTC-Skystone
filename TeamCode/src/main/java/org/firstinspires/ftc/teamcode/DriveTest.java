package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous
@Disabled
public class DriveTest extends LinearOpMode {

    HardwareBot robot;

    public void runOpMode() throws InterruptedException {

        robot = new HardwareBot(this);

        waitForStart();

        //robot.drive.moveDistance(2,0.2,true);
        robot.drive.testMethods();

    }
}
