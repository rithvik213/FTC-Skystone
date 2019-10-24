package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous
public class FoundationTest extends LinearOpMode {

    HardwareBot robot;

    public void runOpMode() throws InterruptedException {

        robot = new HardwareBot(this);
        robot.drive.sLeft.setPosition(0);
        robot.drive.sRight.setPosition(1);

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

        //robot.drive.PTurnIMU(90, 0.3);

        //robot.drive.turnIMU(90,0.4);

        //robot.drive.foundationClamp();
        //robot.drive.moveDistance(10,0.3,true);
        robot.drive.moveDistance2(14,0.2,-0.2,false);
        sleep(1000);
        robot.drive.foundationClamp();
        sleep(1000);
        double target = robot.drive.rightBack.getCurrentPosition() + Math.round(Math.abs(4) * robot.drive.COUNTS_PER_INCH);
        telemetry.addData("target pos", target);
        telemetry.update();
        robot.drive.moveDistance(14,0.2,false);
        //robot.drive.moveDistance(3,0.2,false);
        //robot.drive.turnIMU(40,0.3);

    }
}
