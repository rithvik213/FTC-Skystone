package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
@Autonomous
public class turnIMUTest extends LinearOpMode {

    BNO055IMU imu;
    Orientation angles;

    double angle = 90;
    boolean direction = true;

    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftBack;
    DcMotor rightBack;

    public void runOpMode() throws InterruptedException {

        leftFront = hardwareMap.get(DcMotor.class, "lf");
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack = hardwareMap.get(DcMotor.class, "lb");
        rightBack = hardwareMap.get(DcMotor.class, "rb");
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        waitForStart();

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double startTime = getRuntime();
        double deltaAngle = 0, initTime, deltaTime;
        double i = 0;

        double initialPos = angles.firstAngle;
        double currentPos = initialPos;
        double target = 0;

        if (direction) {
            target = angle - initialPos;
        } else {
            target = angle + initialPos;
        }

        while (opModeIsActive() && ((Math.abs(target - Math.abs(currentPos))) > 1)) {
            initTime = getRuntime();
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentPos = Math.abs(angles.firstAngle);

            double power = Range.clip((Math.abs((currentPos - target) / (100.0)) + i), .3, .7);

            telemetry.addData("Current Position: ", currentPos);
            telemetry.update();
            telemetry.addData("Distance to go: ", (Math.abs(target - Math.abs(currentPos))));
            telemetry.update();

            if (direction) {
                leftFront.setPower(-power);
                //leftBack.setPower(-power);
                rightFront.setPower(power);
                //rightBack.setPower(power);
            } else {
                leftFront.setPower(power);
                //leftBack.setPower(power);
                rightFront.setPower(-power);
                rightBack.setPower(-power);
            }

            deltaTime = getRuntime() - initTime;

            if (Math.abs(currentPos - target) < 30)
                i += .01 * Math.abs(currentPos - target) * deltaTime;

            if (i > 0.3) {
                i = 0.3;
            }
        }
    }
}
