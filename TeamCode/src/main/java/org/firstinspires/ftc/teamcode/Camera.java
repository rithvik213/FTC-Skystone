package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.Locale;
import java.util.Timer;

import static java.lang.String.*;

public class Camera {
    private final LinearOpMode opMode;

    //DogeCV init stuff
    OpenCvCamera phoneCam;
    SkystoneDetector skyStoneDetector;

    public Camera(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void DogeCV() {
        OpenCvCamera phoneCam;
        SkystoneDetector skyStoneDetector;
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        skyStoneDetector = new SkystoneDetector();
        phoneCam.setPipeline(skyStoneDetector);

        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        opMode.waitForStart();

        while (opMode.opModeIsActive()) {
            /*
             * Send some stats to the telemetry
             */
            opMode.telemetry.addData("Stone Position X", skyStoneDetector.getScreenPosition().x);
            opMode.telemetry.addData("Stone Position Y", skyStoneDetector.getScreenPosition().y);
            opMode.telemetry.addData("Frame Count", phoneCam.getFrameCount());
            opMode.telemetry.addData("FPS", format(Locale.US, "%.2f", phoneCam.getFps()));
            opMode.telemetry.addData("Total frame time ms", phoneCam.getTotalFrameTimeMs());
            opMode.telemetry.addData("Pipeline time ms", phoneCam.getPipelineTimeMs());
            opMode.telemetry.addData("Overhead time ms", phoneCam.getOverheadTimeMs());
            opMode.telemetry.addData("Theoretical max FPS", phoneCam.getCurrentPipelineMaxFps());
            opMode.telemetry.update();
        }
    }

    public double[] findSkyStone() {
        initDogeCV();
        boolean SkyStoneFound = skyStoneDetector.isDetected();
        double[] x_y = {0,0};

        while (!SkyStoneFound) {
            SkyStoneFound = skyStoneDetector.isDetected();
            opMode.telemetry.addData("X-Position: ", skyStoneDetector.getScreenPosition().x);
            opMode.telemetry.addData("Y-Position: ", skyStoneDetector.getScreenPosition().y);
            opMode.telemetry.addData("Skystone found: ", skyStoneDetector.isDetected());
            opMode.telemetry.update();
        }
        x_y[0] = skyStoneDetector.getScreenPosition().x;
        x_y[1] = skyStoneDetector.getScreenPosition().y;

        opMode.telemetry.addData("X-Position: ", x_y[0]);
        opMode.telemetry.addData("Y-Position: ", x_y[1]);
        opMode.telemetry.addData("Skystone found: ", skyStoneDetector.isDetected());
        opMode.telemetry.update();

        return x_y;
    }

    public void initDogeCV() {
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        skyStoneDetector = new SkystoneDetector();
        phoneCam.setPipeline(skyStoneDetector);
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
    }
}
