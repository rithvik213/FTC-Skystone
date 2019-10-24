package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous
public class XY_Test extends LinearOpMode {
    HardwareBot robot;
    public void runOpMode() throws InterruptedException {
        robot = new HardwareBot(this);

        robot.drive.getIMUReady();
        robot.drive.resetEncoders();
        robot.drive.runUsingEncoders();

        waitForStart();

        robot.drive.goStraight(0.3);

        while (getRuntime() < 5 && !isStopRequested()) { robot.drive.getXYlocation(); }

        robot.drive.goStraight(0);
    }
}
