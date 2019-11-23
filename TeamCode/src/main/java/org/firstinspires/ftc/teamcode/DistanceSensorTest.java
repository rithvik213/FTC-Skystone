package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensorTest extends LinearOpMode {

    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftBack;
    DcMotor rightBack;

    double WHEEL_DIAMETER_INCHES = 4.0;
    double COUNTS_PER_MOTOR_REV = 1120;
    double GEAR_REDUCTION = 1.5;

    double distance = 20;
    double power  = 0.4;

    private DistanceSensor sensorRange;

    public void runOpMode() throws InterruptedException {

        leftFront = hardwareMap.get(DcMotor.class, "lf");
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack = hardwareMap.get(DcMotor.class, "lb");
        rightBack = hardwareMap.get(DcMotor.class, "rb");
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");

        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;

        double target;
        double counts;

        counts = ((distance / (GEAR_REDUCTION * WHEEL_DIAMETER_INCHES * 3.1415))) * COUNTS_PER_MOTOR_REV;
        target = Math.abs(rightFront.getCurrentPosition()) +  counts;

        while (Math.abs(Math.abs(rightFront.getCurrentPosition()) - Math.abs(target)) > 45 && !isStopRequested()) {
            leftFront.setPower(-power);
            leftBack.setPower(-power);
            rightFront.setPower(-power);
            rightBack.setPower(-power);
            telemetry.addData("Current Position", Math.abs(rightFront.getCurrentPosition()));
            telemetry.addData("Distance to go", Math.abs(Math.abs(rightFront.getCurrentPosition()) - Math.abs(target)));
            telemetry.update();

            if(sensorRange.getDistance(DistanceUnit.INCH) < 5) {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightFront.setPower(0);
                rightBack.setPower(0);
            }
        }
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

    }
}
