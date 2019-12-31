package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.Arrays;

public class MecanumDrive  {
    private final LinearOpMode opMode;
    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftBack = null;
    DcMotor rightBack = null;


    public MecanumDrive(LinearOpMode mode) {
        this.opMode = mode;
        leftFront = opMode.hardwareMap.get(DcMotor.class, "lf");
        rightFront = opMode.hardwareMap.get(DcMotor.class, "rf");
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack = opMode.hardwareMap.get(DcMotor.class, "lb");
        rightBack = opMode.hardwareMap.get(DcMotor.class, "rb");
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void TeleOpDriveMode() {

        double x = opMode.gamepad1.left_stick_x;
        double y = -opMode.gamepad1.left_stick_y;
        double turn = opMode.gamepad1.right_stick_x;

        double leftFrontVal = y + turn + x;
        double leftBackVal = y + turn - x;
        double rightFrontVal = y - turn - x;
        double rightBackVal = y - turn + x;

        /*double leftFrontVal =  opMode.gamepad1.left_stick_y - (opMode.gamepad1.left_stick_x)  + - opMode.gamepad1.right_stick_x;
        double rightFrontVal =  opMode.gamepad1.left_stick_y  + (opMode.gamepad1.left_stick_x) - -opMode.gamepad1.right_stick_x;
        double leftBackVal = opMode.gamepad1.left_stick_y  + (opMode.gamepad1.left_stick_x)  + -opMode.gamepad1.right_stick_x;
        double rightBackVal = opMode.gamepad1.left_stick_y - (opMode.gamepad1.left_stick_x) - -opMode.gamepad1.right_stick_x;
        */

        //Move range to between 0 and +1, if not already
        double[] wheelPowers = {rightFrontVal, leftFrontVal, leftBackVal, rightBackVal};
        Arrays.sort(wheelPowers);
        if (wheelPowers[3] > 1) {
            leftFrontVal /= wheelPowers[3];
            rightFrontVal /= wheelPowers[3];
            leftBackVal /= wheelPowers[3];
            rightBackVal /= wheelPowers[3];
        }

        leftFront.setPower(leftFrontVal);
        rightFront.setPower(rightFrontVal);
        leftBack.setPower(leftBackVal);
        rightBack.setPower(rightBackVal);
    }
}
