package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.List;

@Autonomous
@Disabled
public class Camera_Test extends LinearOpMode {

    //HardwareBot robot;
    //OpenCvCamera phoneCam;
    //SkystoneDetector skyStoneDetector;

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private static final String VUFORIA_KEY =
            "AYQvf/f/////AAABmcM5fT2Yy0GOrmEVG74gUtZSEffMESAYuPiznHckoXw6M6vmUg7+ObKP8fD7rzkw4oJUUbbMn7w4DBn/bRfpVwpjHgLBlaZM97l/v0GcDsA8PnjO5l6E0QhV5DHPm77J+Mtu2nOUSghVdT3qjiWx3Sip+QcraxJxTqVASxktIFH3QjW/8nS7tl4ThoI4mgDqrEFEDMMA/75GF4MsMUZVUbhd6vYIDHy5NzhkakO3t/H+SLZJpggRmW/r/z4V+sTd/xHrcUCaMN0DjomwxvMV5Atu3oMBphQAK622kbHRVYLDR1T2sVWNllRWB8JzgKl7TA9QxiQrajS4OmrwYlfZApA6Wqpibq9NPpBloBn2rhsp";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    double leftPos, rightPos = 0;
    int position = 0;

    //List<Recognition> updatedRecognitions;

    boolean skystoneRecognized = false;


    public void runOpMode() throws InterruptedException {

        initVuforia();

        initTfod();

        if (tfod != null) {
            tfod.activate();
        }

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());

                            if(recognition.getLabel() == "Skystone") {
                                leftPos = recognition.getLeft();
                                rightPos = recognition.getRight();
                                skystoneRecognized = true;
                            }

                            telemetry.addData("leftPos: ", leftPos);
                            telemetry.addData("rightPos: ", rightPos);
                            telemetry.update();
                        }
                        telemetry.update();
                    }
                }
            }

            telemetry.addData("leftPos: ", leftPos);
            telemetry.addData("rightPos: ", rightPos);
            telemetry.update();

            if((leftPos >= 300 && leftPos <= 310) && (rightPos >= 460 && rightPos <= 475)) {
                position = 5;
            }
            else if((leftPos >= 335 && leftPos <= 345) && (rightPos >= 260 && rightPos <= 275)) {
                position = 6;
            }
            else {
                position = 4;
            }

            telemetry.addData("Position: ", position);
            telemetry.update();
        }

        if (tfod != null) {
            tfod.shutdown();
        }
        //robot = new HardwareBot(this);

        //initDogeCV();

        //waitForStart();

       // robot.camera.findSkyStone();
        //findSkyStone();


    }

    /*public void initDogeCV() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        skyStoneDetector = new SkystoneDetector();
        phoneCam.setPipeline(skyStoneDetector);
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
    }

    public double[] findSkyStone() {
        //initDogeCV();
        boolean SkyStoneFound = skyStoneDetector.isDetected();
        double[] x_y = {0,0};

        while (opModeIsActive()) {
            SkyStoneFound = skyStoneDetector.isDetected();
            telemetry.addData("X-Position: ", skyStoneDetector.getScreenPosition().x);
            telemetry.addData("Y-Position: ", skyStoneDetector.getScreenPosition().y);
            telemetry.addData("Skystone found: ", !skyStoneDetector.isDetected());
            telemetry.update();
        }
        x_y[0] = skyStoneDetector.getScreenPosition().x;
        x_y[1] = skyStoneDetector.getScreenPosition().y;

        telemetry.addData("X-Position: ", x_y[0]);
        telemetry.addData("Y-Position: ", x_y[1]);
        telemetry.addData("Skystone found: ", skyStoneDetector.isDetected());
        telemetry.update();

        return x_y;
    }*/

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
