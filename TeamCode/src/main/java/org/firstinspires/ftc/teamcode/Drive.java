package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
@Disabled
public class Drive {

    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftBack;
    DcMotor rightBack;

    private final LinearOpMode opMode;

    BNO055IMU imu;

    double distance = 0;
    double xlocation = 0;
    double ylocation = 0;

    public Drive(LinearOpMode mode) {
        this.opMode = mode;
        leftFront = mode.hardwareMap.get(DcMotor.class, "lf");
        rightFront = mode.hardwareMap.get(DcMotor.class, "rf");
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack = mode.hardwareMap.get(DcMotor.class, "lb");
        rightBack = mode.hardwareMap.get(DcMotor.class, "rb");
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void moveDistance(int distance, double power, boolean direction) {
        resetEncoders();
        runUsingEncoders();

        double WHEEL_DIAMETER_INCHES = 4.0;
        double COUNTS_PER_MOTOR_REV = 1680;
        double GEAR_REDUCTION = 1.0;
        double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
        double target = leftFront.getCurrentPosition() + Math.round(Math.abs(distance) * COUNTS_PER_INCH);

        if (direction) {
            while ((leftFront.getCurrentPosition() - target) > 45 && !opMode.isStopRequested()) {
                setPowerSides(0.4);
                opMode.telemetry.addData("Current Position", leftFront.getCurrentPosition());
                opMode.telemetry.addData("Distance to go", target - leftFront.getCurrentPosition());
                opMode.telemetry.update();
            }
        } else {
            while ((Math.abs(leftFront.getCurrentPosition()) - target) > 45 && !opMode.isStopRequested()) {
                setPowerSides(-0.4);
                opMode.telemetry.addData("Current Position", leftFront.getCurrentPosition());
                opMode.telemetry.addData("Distance to go", target - leftFront.getCurrentPosition());
                opMode.telemetry.update();

            }

        }
        setPowerSides(0.0);
    }

    public void setPowerSides(double power) {
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);
    }

    public void resetEncoders() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runUsingEncoders() {
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void testMethods() {
        leftBack.setPower(-0.2);
        rightBack.setPower(-0.2);
        opMode.sleep(2000);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    public void getXYlocation() {
        getIMUReady();
        double startPos_L = leftFront.getCurrentPosition();
        double startPos_R = rightFront.getCurrentPosition();
        Orientation angles;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        distance = (leftFront.getCurrentPosition() - startPos_L + (rightFront.getCurrentPosition() - startPos_R)) / 2;
        xlocation += distance * Math.cos(angles.firstAngle);
        ylocation += distance * Math.sin(angles.firstAngle);

    }

    public void getIMUReady() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

    }

    public void turnIMU(double angle, double speed) {
        getIMUReady();
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double initialPos = angles.firstAngle;
        double currentPos = initialPos;

        double target = angle + initialPos;

        while(opMode.opModeIsActive() && ((Math.abs(target - currentPos)) > 3)) {
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentPos = Math.abs(angles.firstAngle);

            opMode.telemetry.addData("Current Position: ", currentPos);
            opMode.telemetry.addData("Distance to go: ", target - currentPos);
            opMode.telemetry.update();

            leftFront.setPower(speed);
            leftBack.setPower(speed);
            rightFront.setPower(-speed);
            rightBack.setPower(-speed);
        }
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    public void PTurnIMU(double angle, double speed) {
        getIMUReady();
        double P_COEFFICIENT = 0.005;
        double error = 0;
        double output = 0;

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double initialPos = angles.firstAngle;
        double currentPos = initialPos;

        double target = angle + initialPos;
        do {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentPos = Math.abs(angles.firstAngle);
            error = target - (currentPos + initialPos);

            output = Range.clip(error * P_COEFFICIENT,-1,1);

            leftFront.setPower(output);
            leftBack.setPower(output);
            rightFront.setPower(-output);
            rightBack.setPower(-output);

            opMode.telemetry.addData("Distance left: ", error);
            opMode.telemetry.addData("Current Position: ", currentPos);
            opMode.telemetry.addData("Motor Output: ", output);
            opMode.telemetry.update();

        } while (Math.abs(error) > 2 && opMode.opModeIsActive());
    }
}
