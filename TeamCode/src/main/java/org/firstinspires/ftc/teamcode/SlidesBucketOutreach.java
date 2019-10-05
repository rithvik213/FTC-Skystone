package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Disabled
public class SlidesBucketOutreach extends OpMode {

    DcMotor slides;
    Servo bucket;
    float stallPower = .1f;

    @Override
    public void init(){

        slides = hardwareMap.dcMotor.get("slides");
        bucket = hardwareMap.servo.get("bucket");
    }

    @Override
    public void loop(){

        if (gamepad1.dpad_up){
            slides.setPower(.6);
        } else if (gamepad1.dpad_down){
            slides.setPower(-.3);
        } else {
            slides.setPower(stallPower);
        }

        if (gamepad1.a){
            bucket.setPosition(.25);
        } else {
            bucket.setPosition(0);
        }

    }
}
