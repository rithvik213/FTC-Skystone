package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
@Disabled
public class SlidesTest extends LinearOpMode {

    HardwareBot robot;
    DcMotor slides = null;
    Servo bucket = null;
    float stallPower = .1f;
    double distance  = 4;

    public void runOpMode() throws InterruptedException {

        robot = new HardwareBot(this);

        //slides = hardwareMap.dcMotor.get("slides");
        //bucket = hardwareMap.servo.get("bucket");

        waitForStart();

        double PULLEY_DIAMETER_INCHES = 2.5;
        double COUNTS_PER_MOTOR_REV = 1680;
        double GEAR_REDUCTION = 1.0 ;
        double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * GEAR_REDUCTION) / (PULLEY_DIAMETER_INCHES * 3.1415);
        double target = Math.round(-distance * COUNTS_PER_INCH);

        //telemetry.addData("Target value: %d", target);
        //telemetry.update();

        slides.getCurrentPosition();

        while(slides.getCurrentPosition() > target && opModeIsActive()) {
            telemetry.addData("Slides Current Position: %d", slides.getCurrentPosition());
            telemetry.addData("Target value: %d", target);
            telemetry.update();
            slides.setPower(0.4);
        }
        slides.setPower(0);

        /*slides.setPower(.4);
        sleep(2500);
        slides.setPower(stallPower);
        sleep(1000);

        bucket.setPosition(0);
        sleep(1000);
        bucket.setPosition(.25);
        sleep(1000);


         */

        //robot.extend(2,0.4);

        stop();

    }

}
