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
        while (timer.seconds() <= 5 && opModeIsActive()) {
            telemetry.addData("Pattern: ",robot.camera.getPattern());
            pattern = robot.camera.getPattern();
            telemetry.addData("Timer: ", timer.seconds());
            telemetry.update();
        }

        //robot.drive.moveDistance(2,0.3,true);
        //robot.drive.turnIMU(90,0.4,false);

        switch (pattern) {
            case 1:
                robot.drive.moveDistance(2,0.4,true);
                robot.drive.turnIMU(35,0.4,false);
                robot.drive.moveDistance(27,0.4,true);
                break;

            case 2:
                robot.drive.moveDistance(28, 0.4, true);
                break;

            case 3:
                robot.drive.moveDistance(2, 0.4, true);
                robot.drive.turnIMU(10, 0.4, true);
                robot.drive.moveDistance(27, 0.4, true);
                break;

            default:
                telemetry.addData("No skystone found: ", pattern);
        }
    }
}
