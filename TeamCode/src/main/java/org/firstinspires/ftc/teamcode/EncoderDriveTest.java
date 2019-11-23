package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class EncoderDriveTest extends LinearOpMode {
    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftBack;
    DcMotor rightBack;

    double WHEEL_DIAMETER_INCHES = 4.0;
    double COUNTS_PER_MOTOR_REV = 1120;
    double GEAR_REDUCTION = 1.0;
    double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    boolean direction = false;
    double power = 0.4;
    int distance = 10;


    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotor.class, "lf");
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack = hardwareMap.get(DcMotor.class, "lb");
        rightBack = hardwareMap.get(DcMotor.class, "rb");
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        double counts;
        double target;

        counts = ((distance / (GEAR_REDUCTION * WHEEL_DIAMETER_INCHES * 3.1415))) * COUNTS_PER_MOTOR_REV;
        target = Math.abs(rightFront.getCurrentPosition()) - counts;
        //target = -1 * (-rightFront.getCurrentPosition() - Math.round(Math.abs(distance) * COUNTS_PER_INCH));

        while ((Math.abs(rightFront.getCurrentPosition()) - target) > 45 && !isStopRequested()) {
            leftFront.setPower(power);
            leftBack.setPower(power);
            rightFront.setPower(power);
            rightBack.setPower(power);

            telemetry.addData("Target: ",Math.abs(rightFront.getCurrentPosition()) - counts );
            telemetry.addData("Current Position", Math.abs(rightFront.getCurrentPosition()));
            telemetry.addData("Distance to go",(Math.abs(rightFront.getCurrentPosition()) - target));
            telemetry.update();
        }
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        /*if (direction) {
            while (Math.abs(Math.abs(rightFront.getCurrentPosition()) - target) > 45 && !isStopRequested()) {
                leftFront.setPower(-power);
                leftBack.setPower(-power);
                rightFront.setPower(-power);
                rightBack.setPower(-power);
                telemetry.addData("Current Position", Math.abs(rightFront.getCurrentPosition()));
                telemetry.addData("Distance to go", Math.abs(Math.abs(rightFront.getCurrentPosition()) - target));
                telemetry.update();
            }

        }
        else {

            if(rightFront.getCurrentPosition() < 0) {
                target = -1 * (-rightFront.getCurrentPosition() - Math.round(Math.abs(distance) * COUNTS_PER_INCH));

                while (rightFront.getCurrentPosition() > target && !isStopRequested()) {
                    leftFront.setPower(power);
                    leftBack.setPower(power);
                    rightFront.setPower(power);
                    rightBack.setPower(power);

                    telemetry.addData("Target: ",-1 * (-rightFront.getCurrentPosition() - Math.round(Math.abs(distance) * COUNTS_PER_INCH)) );
                    telemetry.addData("Current Position", (rightFront.getCurrentPosition()));
                    telemetry.addData("Distance to go",(-rightFront.getCurrentPosition() + target));
                    telemetry.update();
                }
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightFront.setPower(0);
                rightBack.setPower(0);
            }
            else {
                //target = -1 *(-rightFront.getCurrentPosition() - Math.round(Math.abs(distance) * COUNTS_PER_INCH));
                target = rightFront.getCurrentPosition() + Math.round(Math.abs(distance) * COUNTS_PER_INCH);

                while ((-rightFront.getCurrentPosition() + target) > 45 && !isStopRequested()) {
                leftFront.setPower(power);
                leftBack.setPower(power);
                rightFront.setPower(power);
                rightBack.setPower(power);

                telemetry.addData("Target: ",-1 * (-rightFront.getCurrentPosition() - Math.round(Math.abs(distance) * COUNTS_PER_INCH)) );
                telemetry.addData("Current Position", (rightFront.getCurrentPosition()));
                telemetry.addData("Distance to go",(-rightFront.getCurrentPosition() + target));
                telemetry.update();
            }
        }
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
            }

            }
            }/* else {
            boolean rotation = true;
            while (Math.abs(target + rightBack.getCurrentPosition()) > 45 && !isStopRequested()) {
                if (rotation) {
                    leftFront.setPower(-power);
                    leftBack.setPower(-power);
                    rightFront.setPower(power*5);
                    rightBack.setPower(power*5);
                    //rotation = false;
                } else {
                    //leftFront.setPower(-0.2);
                    //leftBack.setPower(-0.2);
                    rightFront.setPower(0.2);
                    rightBack.setPower(0.2);
                    rotation = true;

                }

                opMode.telemetry.addData("Current Position", rightBack.getCurrentPosition());
                opMode.telemetry.addData("Distance to go", Math.abs(-rightBack.getCurrentPosition() - target));
                opMode.telemetry.update();
            }*/
    }
}
