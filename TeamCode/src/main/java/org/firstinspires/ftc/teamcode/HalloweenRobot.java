package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.android.dx.dex.code.DalvCode;
@Autonomous
public class HalloweenRobot extends LinearOpMode {

    DcMotor arm;
    //ModernRoboticsI2cRangeSensor range;
    //I2cDeviceSynch rangeAreader;
    int ticks = 1680;
    double target = 0;
    //boolean aPressed = false;
    boolean yPressed = true;


    public void runOpMode() throws InterruptedException {
        arm = hardwareMap.dcMotor.get("arm");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class,"range");


        waitForStart();

        while(opModeIsActive()) {

            //one method to control collecion
            yPressed = gamepad1.y;
            telemetry.addData("Gamepad 1: ",gamepad1.y);
            telemetry.update();
            if (gamepad1.y) {
                //code while y is pressed
                yPressed = false;
                arm.setPower(0.2);
                telemetry.addData("Button Y is pressed", !yPressed);
                telemetry.update();
            } else {
                //code after y is released
                arm.setPower(0);
            }
            /*//telemetry.addData("Range: ", range.getDistance(DistanceUnit.CM));
            telemetry.addData("Encoder ticks: ", Math.abs(arm.getCurrentPosition()));
            telemetry.update();
            if(gamepad1.a) {
                target = Math.abs(arm.getCurrentPosition()) + 180;
                while(Math.abs(arm.getCurrentPosition())<= target) {
                    arm.setPower(-0.2);
                    telemetry.addData("Encoder ticks: ", Math.abs(arm.getCurrentPosition()));
                    telemetry.update();
                }
                arm.setPower(0);
            }

            else if(gamepad1.y) {
                target = Math.abs(arm.getCurrentPosition()) + 180;
                while(Math.abs(arm.getCurrentPosition())<= target) {
                    arm.setPower(0.2);
                    telemetry.addData("Encoder ticks: ", Math.abs(arm.getCurrentPosition()));
                    telemetry.update();
                }
                arm.setPower(0);
            }
            else {
                arm.setPower(0.01);
            }*/
        }


    }
}
