package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
@Disabled
public class Slides {

    DcMotor slides = null;
    private final LinearOpMode opMode;

    public Slides(LinearOpMode mode) {

        slides = mode.hardwareMap.get(DcMotor.class, "slides");
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.opMode = mode;
    }

    public void extendDistance(double distance, double power) {
        double PULLEY_DIAMETER_INCHES = 2.5;
        double COUNTS_PER_MOTOR_REV = 1680;
        double GEAR_REDUCTION = 1.0 ;
        double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * GEAR_REDUCTION) / (PULLEY_DIAMETER_INCHES * 3.1415);
        double target = Math.round(-distance * COUNTS_PER_INCH);

        slides.getCurrentPosition();

        while(slides.getCurrentPosition() > target && opMode.opModeIsActive()) {
            opMode.telemetry.addData("Slides Current Position: %d", slides.getCurrentPosition());
            opMode.telemetry.addData("Target value: %d", target);
            opMode.telemetry.update();
            slides.setPower(0.4);
        }
        slides.setPower(0);
    }

    public void extendTest() {
        slides.setPower(0.4);
        opMode.sleep(2000);
        slides.setPower(0);
    }
}
