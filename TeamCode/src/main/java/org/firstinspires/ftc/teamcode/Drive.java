package org.firstinspires.ftc.teamcode;
import android.util.Pair;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.lang.reflect.Array;

@Disabled
public class Drive {

    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftBack;
    DcMotor rightBack;

    Servo sLeft = null;
    Servo sRight = null;

    private final LinearOpMode opMode;

    BNO055IMU imu;
    Orientation angles;


    double distance = 0;
    double xlocation = 0;
    double ylocation = 0;

    double WHEEL_DIAMETER_INCHES = 4.0;
    double COUNTS_PER_MOTOR_REV = 1120;
    double GEAR_REDUCTION = 1.0;
    double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    public Drive(LinearOpMode mode) {
        this.opMode = mode;
        leftFront = mode.hardwareMap.get(DcMotor.class, "lf");
        rightFront = mode.hardwareMap.get(DcMotor.class, "rf");
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack = mode.hardwareMap.get(DcMotor.class, "lb");
        rightBack = mode.hardwareMap.get(DcMotor.class, "rb");
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        //sLeft = mode.hardwareMap.get(Servo.class, "sleft");
        //sRight = mode.hardwareMap.get(Servo.class, "sright");
    }

    public void moveDistance(int distance, double power, boolean direction) {
        resetEncoders();
        runUsingEncoders();

        double target = rightFront.getCurrentPosition() + Math.round(Math.abs(distance) * COUNTS_PER_INCH);

        if (direction) {
            while (Math.abs(target - Math.abs(rightFront.getCurrentPosition())) > 45 && !opMode.isStopRequested()) {
                goStraight(power);
                opMode.telemetry.addData("Current Position", Math.abs(rightFront.getCurrentPosition()));
                opMode.telemetry.addData("Distance to go", Math.abs(target - Math.abs(rightFront.getCurrentPosition())));
                opMode.telemetry.update();
            }
        } else {
            boolean rotation = true;
            while (Math.abs(target + rightBack.getCurrentPosition()) > 45 && !opMode.isStopRequested()) {
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
            }
        }
        setPowerSides(0.0);
    }

    public void moveDistance2(int distance, double leftpower, double rightpower, boolean direction) {
        resetEncoders();
        runUsingEncoders();

        double target = leftFront.getCurrentPosition() + Math.round(Math.abs(distance) * COUNTS_PER_INCH);

            while (Math.abs(target - leftFront.getCurrentPosition()) > 45 && !opMode.isStopRequested()) {
                leftFront.setPower(leftpower);
                leftBack.setPower(leftpower);
                rightFront.setPower(rightpower);
                rightBack.setPower(rightpower);
                opMode.telemetry.addData("Current Position", leftFront.getCurrentPosition());
                opMode.telemetry.addData("Distance to go", target + leftFront.getCurrentPosition());
                opMode.telemetry.update();
            }
        setPowerSides(0.0);
    }

    public void setPowerSides(double power) {
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);
    }

    public void goStraight(double power) {
        leftFront.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(power);
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

    public double[] getXYlocation() {
        /*
        getIMUReady();
        resetEncoders();
        runUsingEncoders();
        */
        double startPos_L = leftFront.getCurrentPosition();
        double startPos_R = rightFront.getCurrentPosition();
        double[] coordinates = {0,0};
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        distance = ((leftFront.getCurrentPosition() - startPos_L) + (rightFront.getCurrentPosition() - startPos_R)) / 2.0;
        distance/=COUNTS_PER_MOTOR_REV;

        opMode.telemetry.addData("Distance Traveled", distance);

        xlocation += distance * Math.cos(angles.firstAngle);
        coordinates[0] = xlocation;
        opMode.telemetry.addData("X_Loc:", coordinates[0]);
        ylocation += distance * Math.sin(angles.firstAngle);
        coordinates[1] = ylocation;
        opMode.telemetry.addData("Y_Loc:", coordinates[1]);
        opMode.telemetry.update();

        return coordinates;
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

    public void turnIMU(double angle, double speed, boolean direction) {
        getIMUReady();
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double initialPos = angles.firstAngle;
        double currentPos = initialPos;

        double target = angle + initialPos;

        while(opMode.opModeIsActive() && ((Math.abs(target - currentPos)) > 3)) {
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentPos = Math.abs(angles.firstAngle);

            opMode.telemetry.addData("Current Position: ", currentPos);
            opMode.telemetry.update();
            opMode.telemetry.addData("Distance to go: ", target - currentPos);
            opMode.telemetry.update();

            if(direction) {
                leftFront.setPower(speed+0.1);
                //leftBack.setPower(speed);
                rightFront.setPower(-speed);
                //rightBack.setPower(-speed);
            } else {
                leftFront.setPower(-speed-0.1);
                //leftBack.setPower(-speed);
                rightFront.setPower(speed);
                //rightBack.setPower(speed);
            }
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

        } while (Math.abs(error) > 1 && opMode.opModeIsActive());
    }

   /* public  void goToPosition(final double x, final double y, final double movementSpeed, final double preferredAngle, final double turnSpeed) {
        final double distanceToTarget = Math.hypot(x - xlocation, y - ylocation);
        final double absoluteAngleToTarget = Math.atan2(y - ylocation, x - xlocation);
        final double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (worldAngle_rad - Math.toRadians(90.0)));
        final double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        final double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;
        final double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        final double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        movement_x = movementXPower * movementSpeed;
        movement_y = movementYPower * movementSpeed;
        final double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180.0) + preferredAngle;
        movement_turn = Range.clip(relativeTurnAngle / Math.toRadians(30.0), -1.0, 1.0) * turnSpeed;
        if (distanceToTarget < 10.0) {
            movement_turn = 0.0;
        }

    }
*/
    public  double AngleWrap(double angle) {
        while (angle < -180) {
            angle += 2 * 180;
        }

        while (angle > 180) {
            angle-= 2 * 180;
        }

        return angle;
    }

    public void foundationClamp() {
        sLeft.setPosition(0.4);
        sRight.setPosition(0.5);
    }


}
