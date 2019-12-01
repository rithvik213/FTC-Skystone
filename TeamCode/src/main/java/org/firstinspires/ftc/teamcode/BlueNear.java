package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
@Autonomous
public class BlueNear extends LinearOpMode {
    HardwareBot robot;
    ElapsedTime timer;
    int pattern  = 0;

    public void runOpMode() throws InterruptedException {
        robot = new HardwareBot(this);
        timer = new ElapsedTime();
        robot.camera.setAllianceColor("blue");

        waitForStart();

        timer.reset();
        while (timer.seconds() <= 2 && !isStopRequested()) {
            telemetry.addData("Pattern: ",robot.camera.getPattern());
            pattern = robot.camera.getPattern();
            telemetry.addData("Timer: ", timer.seconds());
            telemetry.update();
        }

        switch (pattern) {
            case 1:
                robot.accessories.readyToGrabOrUnlatch();
                robot.drive.turnIMUOneSide(12,0.3,true);
                robot.drive.moveDistance(21,0.4,true);
                sleep(700);
                robot.drive.turnIMUOneSide(12,0.3,false);
                robot.accessories.Grab();
                sleep(700);
                robot.drive.moveDistance(13,0.4, false);
                robot.drive.turnIMU(82,0.4,false); // initially false before 85
                robot.drive.moveDistance(58,0.4,true);  // initially true
                robot.accessories.readyToGrabOrUnlatch();
                sleep(700);
                robot.drive.moveDistance(8,0.4,false);


                //robot.drive.turnIMUOneSide(40,0.3,true);
                //robot.drive.moveDistance(43,0.4,true);
                break;

            case 2:
                robot.accessories.readyToGrabOrUnlatch();
                robot.drive.turnIMUOneSideBlue(31,0.3,true); //prev 37
                robot.drive.moveDistance(36,0.4,true);
                sleep(700);
                robot.drive.turnIMUOneSideBlue(31,0.3,false);
                robot.accessories.Grab();
                sleep(700);
                robot.drive.moveDistance(10,0.4, false);
                //PREVIOUS PIVOT TURN
                robot.drive.turnIMU(84,0.3,false);
                //robot.drive.turnIMUThreeWheel(90,0.3,true);
                robot.drive.moveDistance(73,0.4,true);
                robot.accessories.readyToGrabOrUnlatch();
                sleep(700);
                robot.drive.moveDistance(62,0.4,false);
                robot.drive.turnIMUOneSide(89,0.4,true);
                robot.drive.moveDistance(9,0.4,true);
                robot.accessories.Grab();
                sleep(700);
                robot.drive.moveDistance(13,0.4,false);
                robot.drive.turnIMU(87,0.4,false);
                robot.drive.moveDistance(49,0.4,true);
                robot.accessories.readyToGrabOrUnlatch();
                sleep(700);
                robot.drive.moveDistance(7,0.4,false);
                /*robot.accessories.readyToGrabOrUnlatch();
                robot.drive.turnIMUOneSide(37.5,0.3,true); //prev 37
                robot.drive.moveDistance(23.5,0.4,true);
                sleep(700);
                robot.drive.turnIMUOneSide(36,0.3,false);
                robot.accessories.Grab();
                sleep(700);
                robot.drive.turnIMU(17,0.3,true);
                robot.drive.moveDistance(11,0.4, false);
                robot.drive.turnIMU(100,0.3,false);
                //NEED TO TEST
                /*robot.drive.turnIMU(17,0.3,true);
                robot.drive.moveDistance(11,0.4, false);
                robot.drive.turnIMU(82,0.4,false); // initially false before 85
                 */
                /*
                robot.drive.moveDistance(68,0.4,true);  // initially true
                robot.accessories.readyToGrabOrUnlatch();
                sleep(700);
                robot.drive.moveDistance(56,0.4,false); //false before
                robot.drive.turnIMUOneSide(91,0.4,true); // true
                robot.drive.moveDistance(7,0.4,true);
                robot.accessories.Grab();
                sleep(700);
                robot.drive.moveDistance(9,0.4,false);
                robot.drive.turnIMU(83,0.4,false); //false before
                robot.drive.moveDistance(47,0.4,true); //true before
                robot.accessories.readyToGrabOrUnlatch();
                sleep(700);
                */
                break;

            case 3:
                robot.accessories.readyToGrabOrUnlatch();
                robot.drive.turnIMUOneSide(26,0.4,true);
                sleep(700);
                robot.drive.moveDistance(22,0.4,true);
                robot.drive.turnIMUOneSide(26,0.4,false);
                robot.accessories.Grab();
                sleep(700);
                robot.drive.moveDistance(16,0.4, false);
                robot.drive.turnIMUOneSideBlue(78,0.3,false);
                robot.drive.moveDistance(66,0.4,true);
                robot.accessories.readyToGrabOrUnlatch();
                sleep(700);
                robot.drive.moveDistance(55,0.4,false);
                robot.drive.turnIMUOneSide(84,0.4,true);
                robot.drive.moveDistance(4,0.4,true);
                robot.accessories.Grab();
                sleep(700);
                robot.drive.moveDistance(12,0.4,false);
                robot.drive.turnIMU(78,0.4,false);
                robot.drive.moveDistance(45,0.4,true);
                robot.accessories.readyToGrabOrUnlatch();
                sleep(700);
                robot.drive.moveDistance(7,0.4,false);
                break;

            default:
                telemetry.addData("No skystone found: ", pattern);
        }
    }
}
