package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous
public class FoundationTest extends LinearOpMode {

    HardwareBot robot;

    public void runOpMode() throws InterruptedException {

        robot = new HardwareBot(this);

        waitForStart();

        /*robot.drive.leftFront.setPower(0.6);
        robot.drive.leftBack.setPower(0.6);
        robot.drive.rightFront.setPower(0.3);
        robot.drive.rightBack.setPower(0.3);

        sleep(2500);

        robot.drive.leftFront.setPower(0);
        robot.drive.leftBack.setPower(0);
        robot.drive.rightFront.setPower(0);

        }
        robot.drive.rightBack.setPower(0);

        */

        robot.drive.PTurnIMU(90, 0.3);

        //robot.drive.turnIMU(90,0.4);

    }
}
