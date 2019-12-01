package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
@Autonomous
public class RedNear extends LinearOpMode {
    HardwareBot robot;
    ElapsedTime timer;
    int pattern  = 0;

    public void runOpMode() throws InterruptedException {
        robot = new HardwareBot(this);
        timer = new ElapsedTime();
        robot.camera.setAllianceColor("red");

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
                //test
                robot.accessories.readyToGrabOrUnlatch();
                robot.drive.turnIMUOneSide(10,0.3,false);
                robot.drive.moveDistance(22,0.4,true);
                sleep(700);
                robot.drive.turnIMUOneSide(10,0.3,true);
                robot.accessories.Grab();
                sleep(700);
                robot.drive.moveDistance(11,0.4, false);
                robot.drive.turnIMU(87,0.4,true); // initially false before 85
                robot.drive.moveDistance(58,0.4,true);  // initially true
                robot.accessories.readyToGrabOrUnlatch();
                sleep(700);
                robot.drive.moveDistance(8,0.4,false);

                //robot.drive.moveDistance(4,0.4,true);
                //robot.drive.turnIMU(14,0.3,false);
                //robot.drive.turnIMUOneSide(43,0.3,false);
                //robot.drive.moveDistance(30,0.4,true);
                //robot.drive.turnIMU(14,0.3,true);
                //sleep(1000);
                //robot.drive.moveDistance(10,0.4,true);
                //robot.accessories.Grab();
                //robot.drive.moveDistance(1,0.3,true);
                //robot.accessories.Grab();
                //robot.drive.turnIMU(90,0.4,true);
                //robot.drive.moveDistance(8,0.4,true);

                /* PREVIOUS PATH WITH ROBOT LINED UP WITH RIGHT OF MAT
                robot.drive.moveDistance(2,0.4,true);
                robot.drive.turnIMU(28,0.55,false);
                robot.drive.moveDistance(25,0.3,true); //prev 27
                 */
                break;

            case 2:
                robot.accessories.readyToGrabOrUnlatch();
                robot.drive.turnIMUOneSide(42,0.3,false); //prev 37
                robot.drive.moveDistance(21,0.4,true);
                sleep(500);
                robot.drive.turnIMUOneSide(42,0.3,true);
                robot.accessories.Grab();
                sleep(700);
                robot.drive.moveDistance(13,0.4, false);
                //PREVIOUS PIVOT TURN
                robot.drive.Auto2ndTurnRed(90,0.3);
                //robot.drive.turnIMU(90,0.3,true);
                //robot.drive.turnIMUThreeWheel(90,0.3,true);
                robot.drive.moveDistance(70,0.4,true);
                robot.accessories.readyToGrabOrUnlatch();
                sleep(700);
                robot.drive.moveDistance(59,0.4,false);
                robot.drive.turnIMUOneSide(89,0.4,false);
                robot.drive.moveDistance(9,0.4,true);
                robot.accessories.Grab();
                sleep(700);
                robot.drive.moveDistance(13,0.4,false);
                robot.drive.turnIMU(87,0.4,true);
                robot.drive.moveDistance(46,0.4,true);
                robot.accessories.readyToGrabOrUnlatch();
                sleep(700);
                robot.drive.moveDistance(4,0.4,false);


                /*robot.drive.moveDistance(30,0.4,true);
                robot.accessories.Grab();
                sleep(2000);
                robot.drive.moveDistance(7,0.4, false);
                robot.drive.turnIMU(90,0.4,true);
                robot.drive.moveDistance(47,0.4,true);
                robot.accessories.readyToGrabOrUnlatch();
                sleep(2000);
                //robot.drive.turnIMU(10,0.4,false);
                robot.drive.moveDistance(60,0.4,false);
                sleep(1000);
                robot.drive.turnIMU(90,0.4,false);
                */

                //robot.drive.moveDistance(8,0.4,false);
                //robot.drive.turnIMU(94,0.4,true);
                //robot.drive.resetEncoders();
                //robot.drive.runWithoutEncoders();
                //robot.drive.moveDistance(10, 0.4, true);
                /* PREVIOUS PATH WITH ROBOT LINED UP WITH RIGHT OF MAT
                robot.drive.moveDistance(10,0.4,true);
                //wait(1000);
                robot.drive.turnIMU(9,0.3,false);
                //wait(1000);
                //robot.drive.moveDistance(22, 0.3, true); //prev 28
                //robot.drive.turnIMU(9,0.3,false);
                //robot.drive.moveDistance(2,0.4,false);
                //robot.drive.turnIMU(90,0.4,false);
                //robot.drive.turnIMU(90,0.4,false);
                 */
                break;

            case 3:
                robot.accessories.readyToGrabOrUnlatch();
                robot.drive.turnIMUOneSide(24,0.4,false);
                robot.drive.moveDistance(18,0.4,true);
                robot.drive.turnIMUOneSide(24,0.4,true);
                robot.accessories.Grab();
                sleep(700);
                robot.drive.moveDistance(13,0.4, false);
                robot.drive.turnIMU(90,0.4,true);
                robot.drive.moveDistance(62,0.4,true);
                robot.accessories.readyToGrabOrUnlatch();
                sleep(700);
                robot.drive.moveDistance(51,0.4,false);
                robot.drive.turnIMUOneSide(91,0.4,false);
                robot.drive.moveDistance(7,0.4,true);
                robot.accessories.Grab();
                sleep(700);
                robot.drive.moveDistance(13,0.4,false);
                robot.drive.turnIMU(85,0.4,true);
                robot.drive.moveDistance(40,0.4,true);
                robot.accessories.readyToGrabOrUnlatch();
                sleep(700);
                robot.drive.moveDistance(7,0.4,false);

                /*robot.drive.moveDistance(4,0.4,true);
                robot.drive.turnIMU(17,0.3,true);
                robot.drive.moveDistance(40,0.4,true);
                robot.accessories.readyToGrabOrUnlatch();
                robot.drive.turnIMU(17,0.4,false);
                //robot.drive.resetEncoders();
                //robot.drive.runWithoutEncoders();
                robot.drive.moveDistance(3,0.4,true);
                robot.accessories.Grab();
                robot.drive.turnIMU(90,0.4,true);
                robot.drive.moveDistance(10,0.4,true);
                */

                /* PREVIOUS PATH WITH ROBOT LINED UP WITH RIGHT OF MAT
                robot.drive.moveDistance(2, 0.4, true);
                robot.drive.turnIMU(10, 0.4, false);
                robot.drive.moveDistance(10, 0.3, true); //prev 27
                 */
                break;

            default:
                telemetry.addData("No skystone found: ", pattern);
        }
    }
}
