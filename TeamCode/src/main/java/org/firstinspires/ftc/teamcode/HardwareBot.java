package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
@Disabled
public class HardwareBot {

    Drive drive;
    Camera camera;
    Accessories accessories;

    private final LinearOpMode opMode1;

    public HardwareBot(LinearOpMode mode) {
        this.opMode1 = mode;
        drive = new Drive(mode);
        camera = new Camera(mode);
        accessories = new Accessories(mode);
    }

}
