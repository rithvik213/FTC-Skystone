package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
public class Accessories {

    Servo autoClutch = null;

    private final LinearOpMode opMode;

    public Accessories(LinearOpMode mode) {
        this.opMode = mode;
        autoClutch = opMode.hardwareMap.servo.get("autoClutch");
        //autoClutch.setPosition(0);

        //Prev auto clutch
       //autoClutch.setPosition(0.8);
        autoClutch.setPosition(1);
    }

    public void readyToGrabOrUnlatch() {
        //autoClutch.setPosition(0.2);
        autoClutch.setPosition(0.4);
    }

    public void Grab() {
        //autoClutch.setPosition(0.45);
        autoClutch.setPosition(0.8);
    }
}
