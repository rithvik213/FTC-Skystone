package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@TeleOp
public class TeleOpTest extends LinearOpMode {

    //HardwareBot robot;
    double left;
    double right;
    double drive;
    double turn;
    double max;

    boolean wentUpOnce = false;

    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftBack;
    DcMotor rightBack;

    //CRServo crservo = null;

    DcMotor slides = null;

    Servo clutch = null;
    double up;
    double down;
    Servo autoClutch = null;

    Deadline gamepadRateLimit;
    private final int GAMEPAD_LOCKOUT = 500;

    int numClickedUp = -1;
    int numClickedDown = -1;

    //double newTarget;
    double counts;

    double WHEEL_DIAMETER_INCHES = 4.0;
    double COUNTS_PER_MOTOR_REV = 1680;
    double GEAR_REDUCTION = 1.5; //should be 1.5



    public void runOpMode() throws InterruptedException {
        //robot = new HardwareBot(this);

        leftFront = hardwareMap.get(DcMotor.class, "lf");
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack = hardwareMap.get(DcMotor.class, "lb");
        rightBack = hardwareMap.get(DcMotor.class, "rb");
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        slides = hardwareMap.dcMotor.get("slides");
        slides.setDirection(DcMotorSimple.Direction.REVERSE);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //need to change to RUN_USING_ENCODER

        clutch = hardwareMap.get(Servo.class,"clutch");
        autoClutch = hardwareMap.servo.get("autoClutch");

        gamepadRateLimit = new Deadline(GAMEPAD_LOCKOUT, TimeUnit.MILLISECONDS);

        waitForStart();

        while (!isStopRequested()) {

            drive = -gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;


            left = drive - turn; //before drive - turn
            right = drive + turn; // before drive + turn


            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0) {
                left /= max;
                right /= max;
            }

            /*WORKED
            double up = -gamepad1.left_stick_y;
            double down = -gamepad1.right_stick_y;
            */

            up = gamepad1.left_trigger + gamepad2.left_trigger;
            down = gamepad1.right_trigger + gamepad2.right_trigger;

            if(gamepad1.right_trigger > 0.1 || gamepad2.right_trigger > 0.1) {
                slides.setPower(0.2);
            }
            else if(gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1) {
                slides.setPower(-0.2);
            }
            else {
                slides.setPower(0);
            }

            leftFront.setPower(-left);
            leftBack.setPower(-left);
            rightFront.setPower(-right);
            rightBack.setPower(-right);

            /*if(gamepad2.right_bumper) {
                slides.setPower(0.3);
            }
            else if(gamepad2.left_bumper) {
                slides.setPower(-0.3);
            } else {
                slides.setPower(0);
            }*/

            if(gamepad1.dpad_up || gamepad2.dpad_up) {
                clutch.setPosition(0.2);

            }
            else if(gamepad1.dpad_down || gamepad2.dpad_down) {
                clutch.setPosition(0.7);
            }

            if(gamepad1.a || gamepad2.a) {
                autoClutch.setPosition(1);
            }

            if(gamepad1.x || gamepad2.x) {
                autoClutch.setPosition(0.4);
            }
            telemetry.addData("Slides Encoder counts:", slides.getCurrentPosition());
            //handleGamePad();
            telemetry.update();


        }


    }

    public void handleGamePad() {

        double newTarget = 0;
        double counts;

//        double WHEEL_DIAMETER_INCHES = 4.0;
//        double COUNTS_PER_MOTOR_REV = 1680;
//        double GEAR_REDUCTION = 1.5; //needs to be changed

        if (!gamepadRateLimit.hasExpired()) {
            return;
        }

        if (gamepad1.right_bumper || gamepad2.right_bumper) {
            numClickedUp++;
            gamepadRateLimit.reset();
            boolean jumpOut = false;

            newTarget = (822 - Math.abs(slides.getCurrentPosition())) + (numClickedUp * 413);

            slides.setTargetPosition( (int) newTarget);

            slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            slides.setPower(-0.2);

            while(slides.isBusy() && opModeIsActive() && !jumpOut) {
                if(gamepad1.b) {
                    slides.setPower(0);
                    jumpOut = true;
                }
                telemetry.addData("Target Position", newTarget);
                telemetry.addData("Current Position: ", Math.abs(slides.getCurrentPosition()));
                telemetry.update();
            }
            slides.setPower(0);
            slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            numClickedUp = -1;
        }
        else if(gamepad1.left_bumper || gamepad2.left_bumper) {
            numClickedDown++;
            gamepadRateLimit.reset();
        }

        telemetry.addData("NumClicks Up: ", numClickedUp);
        telemetry.addData("NumClicks Down: ", numClickedDown);
    }
}
