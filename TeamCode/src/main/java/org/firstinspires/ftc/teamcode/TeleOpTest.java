package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class TeleOpTest extends LinearOpMode {

    //HardwareBot robot;
    double left;
    double right;
    double drive;
    double turn;
    double max;

    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftBack;
    DcMotor rightBack;

    public void runOpMode() throws InterruptedException {
        //robot = new HardwareBot(this);

        leftFront = hardwareMap.get(DcMotor.class, "lf");
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack = hardwareMap.get(DcMotor.class, "lb");
        rightBack = hardwareMap.get(DcMotor.class, "rb");
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            drive = -gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;


            left = drive + turn;
            right = drive - turn;


            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0) {
                left /= max;
                right /= max;
            }

            leftFront.setPower(left);
            leftBack.setPower(left);
            rightFront.setPower(right);
            rightBack.setPower(right);
        }


    }
}
