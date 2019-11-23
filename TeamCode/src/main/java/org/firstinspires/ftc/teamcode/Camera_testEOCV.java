package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class Camera_testEOCV extends LinearOpMode {

    HardwareBot robot;
    ElapsedTime timer;
    int pattern  = 0;

    public void runOpMode() throws InterruptedException {
        robot = new HardwareBot(this);
        timer = new ElapsedTime();

        waitForStart();

        timer.reset();
        while (timer.seconds() <= 3 && opModeIsActive()) {
            telemetry.addData("Pattern: ",robot.camera.getPattern());
            pattern = robot.camera.getPattern();
            telemetry.addData("Timer: ", timer.seconds());
            telemetry.update();
        }

        //robot.drive.moveDistance(2,0.3,true);
        //robot.drive.turnIMU(90,0.4,false);

        switch (pattern) {
            case 1:
                robot.drive.moveDistance(4,0.4,true);
                robot.drive.turnIMU(17,0.3,false);
                robot.drive.moveDistance(33,0.4,true);
                robot.drive.turnIMU(17,0.4,true);
                /* PREVIOUS PATH WITH ROBOT LINED UP WITH RIGHT OF MAT
                robot.drive.moveDistance(2,0.4,true);
                robot.drive.turnIMU(28,0.55,false);
                robot.drive.moveDistance(25,0.3,true); //prev 27
                 */
                break;

            case 2:
                robot.drive.moveDistance(32,0.4,true);
                robot.drive.moveDistance(3,0.4,false);
                robot.drive.turnIMU(82,0.4,true);
                robot.drive.moveDistance(33, 0.4, true);
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
                robot.drive.moveDistance(4,0.4,true);
                robot.drive.turnIMU(17,0.3,true);
                robot.drive.moveDistance(40,0.4,true);
                robot.drive.turnIMU(17,0.4,false);
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
