package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Locale;
import java.util.Timer;

import static java.lang.String.*;
import static org.opencv.core.CvType.CV_8UC1;

public class Camera {
    private final LinearOpMode opMode;

    //DogeCV init stuff
    OpenCvCamera phoneCam;
    SkystoneDetector skyStoneDetector;

    //EasyOpenCV init
    int left_hue;
    int right_hue;

    int left_br;
    int right_br;

    int pattern;

    OpenCvCamera phoneCamera;
    SamplePipeline stone_pipeline;

    String allianceColor = "";

    Scalar left_mean;
    Scalar right_mean;

    public Camera(LinearOpMode opMode) {
        this.opMode = opMode;

        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        phoneCamera = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCamera.openCameraDevice();

        stone_pipeline = new SamplePipeline();
        phoneCamera.setPipeline(stone_pipeline);

        phoneCamera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
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
        double[] x_y = {0, 0};

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

    public void setAllianceColor(String alliance) {
        allianceColor = alliance;
    }

    class SamplePipeline extends OpenCvPipeline {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();


        @Override
        public Mat processFrame(Mat input) {
            //input.convertTo(input, CV_8UC1, 1, 10);
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);

            int[] left_rect;
            int[] right_rect;

            //telemetry.addData("Input Cols: ", input.cols());
            //telemetry.addData("Input Rows: ", input.rows());
            //telemetry.update();

            if (allianceColor.equalsIgnoreCase("blue")) {
                left_rect = new int[]{
                        (int) (input.cols() * (9f / 32f)),
                        (int) (input.rows() * (17f / 32f)),
                        (int) (input.cols() * (17f / 32f)), //previously 11 with 17
                        (int) (input.rows() * (21f / 32f))

                        /*sideways right config
                        (int) (input.cols() * (11f / 32f)),
                        (int) (input.rows() * (12f / 32f)),
                        (int) (input.cols() * (17f / 32f)),
                        (int) (input.rows() * (17f / 32f))
                         */
                };

                right_rect = new int[]{
                        (int) (input.cols() * (17f / 32f)),
                        (int) (input.rows() * (17f / 32f)),
                        (int) (input.cols() * (25f / 32f)), //previously 19 with 25
                        (int) (input.rows() * (21f / 32f))

                        /*sideways right config
                         (int) (input.cols() * (19f / 32f)),
                        (int) (input.rows() * (12f / 32f)),
                        (int) (input.cols() * (25f / 32f)),
                        (int) (input.rows() * (17f / 32f))
                        */
                };
            } else {
                left_rect = new int[]{
                        (int) (input.cols() * (8f / 32f)),
                        (int) (input.rows() * (16f / 32f)),
                        (int) (input.cols() * (16f / 32f)), //previously 11 with 17
                        (int) (input.rows() * (20f / 32f))

                        /*sideways right config
                        (int) (input.cols() * (11f / 32f)),
                        (int) (input.rows() * (12f / 32f)),
                        (int) (input.cols() * (17f / 32f)),
                        (int) (input.rows() * (17f / 32f))
                        */
                };

                right_rect = new int[]{
                        (int) (input.cols() * (16f / 32f)),
                        (int) (input.rows() * (16f / 32f)),
                        (int) (input.cols() * (24f / 32f)), //previously 19 with 25
                        (int) (input.rows() * (20f / 32f))

                        /*sideways right config
                         (int) (input.cols() * (19f / 32f)),
                        (int) (input.rows() * (12f / 32f)),
                        (int) (input.cols() * (25f / 32f)),
                        (int) (input.rows() * (17f / 32f))
                         */
                };
                //If new method doesn't work, use this
                /*left_rect = new int[]{
                        (int) (input.cols() * (9f / 32f)),
                        (int) (input.rows() * (17f / 32f)),
                        (int) (input.cols() * (17f / 32f)), //previously 11 with 17
                        (int) (input.rows() * (21f / 32f))

                        /*sideways right config
                        (int) (input.cols() * (11f / 32f)),
                        (int) (input.rows() * (12f / 32f)),
                        (int) (input.cols() * (17f / 32f)),
                        (int) (input.rows() * (17f / 32f))

                };

                right_rect = new int[]{
                        (int) (input.cols() * (17f / 32f)),
                        (int) (input.rows() * (17f / 32f)),
                        (int) (input.cols() * (25f / 32f)), //previously 19 with 25
                        (int) (input.rows() * (21f / 32f))

                        /*sideways right config
                         (int) (input.cols() * (19f / 32f)),
                        (int) (input.rows() * (12f / 32f)),
                        (int) (input.cols() * (25f / 32f)),
                        (int) (input.rows() * (17f / 32f))

                };
            } else {
                left_rect = new int[]{
                        (int) (input.cols() * (6f / 32f)),
                        (int) (input.rows() * (18f / 32f)),
                        (int) (input.cols() * (14f / 32f)), //previously 11 with 17
                        (int) (input.rows() * (22f / 32f))

                        /*sideways right config
                        (int) (input.cols() * (11f / 32f)),
                        (int) (input.rows() * (12f / 32f)),
                        (int) (input.cols() * (17f / 32f)),
                        (int) (input.rows() * (17f / 32f))

                };

                right_rect = new int[]{
                        (int) (input.cols() * (14f / 32f)),
                        (int) (input.rows() * (18f / 32f)),
                        (int) (input.cols() * (22f / 32f)), //previously 19 with 25
                        (int) (input.rows() * (22f / 32f))

                        /*sideways right config
                         (int) (input.cols() * (19f / 32f)),
                        (int) (input.rows() * (12f / 32f)),
                        (int) (input.cols() * (25f / 32f)),
                        (int) (input.rows() * (17f / 32f))

                };*/
            }
            Imgproc.rectangle(
                    input,
                    new org.opencv.core.Point(
                            left_rect[0],
                            left_rect[1]),

                    new org.opencv.core.Point(
                            left_rect[2],
                            left_rect[3]),
                    new Scalar(0, 255, 0), 1);

            Imgproc.rectangle(
                    input,
                    new org.opencv.core.Point(
                            right_rect[0],
                            right_rect[1]),

                    new Point(
                            right_rect[2],
                            right_rect[3]),
                    new Scalar(0, 0, 255), 1);

            Mat left_block = input.submat(left_rect[1], left_rect[3], left_rect[0], left_rect[2]);
            Mat right_block = input.submat(right_rect[1], right_rect[3], right_rect[0], right_rect[2]);

            left_mean = Core.mean(left_block);


            right_mean = Core.mean(right_block);

            if (allianceColor.equalsIgnoreCase("red")) {

                if (Math.abs(left_mean.val[0] - right_mean.val[0]) <= 25) pattern = 3;
                else if (left_mean.val[0] < right_mean.val[0]) pattern = 1;
                else if (left_mean.val[0] > right_mean.val[0]) pattern = 2;

                //else if (Math.abs((left_mean.val[0] - right_mean.val[0])) <= 10) pattern = 3;
                /*else if (left_br < 90 && right_br < 90) {
                    if (left_br > right_br) {
                        pattern = 1;
                    } else if (left_br < right_br) {
                        pattern = 2;
                    } else {
                        pattern = 3;
                    }
                }*/
            } else {

                if (Math.abs(left_mean.val[0] - right_mean.val[0]) <= 25) pattern = 3;
                else if (left_mean.val[0] < right_mean.val[0]) pattern = 2;
                else if (left_mean.val[0] > right_mean.val[0]) pattern = 1;

                //if (left_mean.val[0] > right_mean.val[0]) pattern = 1;
                //else if (left_mean.val[0] < right_mean.val[0]) pattern = 2;
                //else if(Math.abs(left_mean.val[0] - right_mean.val[0]) <= 25) pattern = 3;
                //else if (Math.abs((left_mean.val[0] - right_mean.val[0])) <= 10) pattern = 3;
                /*else if (left_br < 85 && right_br < 85) {
                    if (left_br > right_br) {
                        pattern = 1;
                    } else if (left_br < right_br) {
                        pattern = 2;
                    } else {
                        pattern = 3;
                    }
                }*/
            }
            return input;
        }

    /*
    class SamplePipeline extends OpenCvPipeline {

        @Override
        public Mat processFrame(Mat input) {
            input.convertTo(input, CV_8UC1, 1, 10);
            int[] left_rect;
            int[] right_rect;

            //telemetry.addData("Input Cols: ", input.cols());
            //telemetry.addData("Input Rows: ", input.rows());
            //telemetry.update();

            if(allianceColor.equalsIgnoreCase("blue")) {
                left_rect = new int[]{
                        (int) (input.cols() * (10f / 32f)),
                        (int) (input.rows() * (16f / 32f)),
                        (int) (input.cols() * (18f / 32f)), //previously 11 with 17
                        (int) (input.rows() * (20f / 32f))

                        /*sideways right config
                        (int) (input.cols() * (11f / 32f)),
                        (int) (input.rows() * (12f / 32f)),
                        (int) (input.cols() * (17f / 32f)),
                        (int) (input.rows() * (17f / 32f))

                };

                right_rect = new int[] {
                        (int) (input.cols() * (18f / 32f)),
                        (int) (input.rows() * (16f / 32f)),
                        (int) (input.cols() * (26f / 32f)), //previously 19 with 25
                        (int) (input.rows() * (20f / 32f))

                        /*sideways right config
                         (int) (input.cols() * (19f / 32f)),
                        (int) (input.rows() * (12f / 32f)),
                        (int) (input.cols() * (25f / 32f)),
                        (int) (input.rows() * (17f / 32f))

                };
            }
            else {
                left_rect = new int[]{
                        (int) (input.cols() * (9f / 32f)),
                        (int) (input.rows() * (17f / 32f)),
                        (int) (input.cols() * (17f / 32f)), //previously 11 with 17
                        (int) (input.rows() * (21f / 32f))

                        /* Previous Red Config Upright
                        (int) (input.cols() * (13f / 32f)),
                        (int) (input.rows() * (15f / 32f)),
                        (int) (input.cols() * (21f / 32f)), //previously 11 with 17
                        (int) (input.rows() * (19f / 32f))
                        */
                        /*sideways right config
                        (int) (input.cols() * (11f / 32f)),
                        (int) (input.rows() * (12f / 32f)),
                        (int) (input.cols() * (17f / 32f)),
                        (int) (input.rows() * (17f / 32f))

                };

                right_rect = new int[] {
                        (int) (input.cols() * (17f / 32f)),
                        (int) (input.rows() * (17f / 32f)),
                        (int) (input.cols() * (25f / 32f)), //previously 19 with 25
                        (int) (input.rows() * (21f / 32f))

                        /* Previous Red Config Upright
                        (int) (input.cols() * (21f / 32f)),
                        (int) (input.rows() * (15f / 32f)),
                        (int) (input.cols() * (29f / 32f)), //previously 19 with 25
                        (int) (input.rows() * (19f / 32f))
                        */

        /*sideways right config
         (int) (input.cols() * (19f / 32f)),
        (int) (input.rows() * (12f / 32f)),
        (int) (input.cols() * (25f / 32f)),
        (int) (input.rows() * (17f / 32f))

};
}
Imgproc.rectangle(
    input,
    new org.opencv.core.Point(
            left_rect[0],
            left_rect[1]),

    new org.opencv.core.Point(
            left_rect[2],
            left_rect[3]),
    new Scalar(0, 255, 0), 1);

Imgproc.rectangle(
    input,
    new org.opencv.core.Point(
            right_rect[0],
            right_rect[1]),

    new Point(
            right_rect[2],
            right_rect[3]),
    new Scalar(0, 0, 255), 1);

Mat left_block = input.submat(left_rect[1], left_rect[3], left_rect[0], left_rect[2]);
Mat right_block = input.submat(right_rect[1], right_rect[3], right_rect[0], right_rect[2]);


Scalar left_mean = Core.mean(left_block);


Scalar right_mean = Core.mean(right_block);

left_hue = get_hue((int) left_mean.val[0], (int) left_mean.val[1], (int) left_mean.val[2]);
right_hue = get_hue((int) right_mean.val[0], (int) right_mean.val[1], (int) right_mean.val[2]);
left_br = get_brightness((int) left_mean.val[0], (int) left_mean.val[1], (int) left_mean.val[2]);
right_br = get_brightness((int) right_mean.val[0], (int) right_mean.val[1], (int) right_mean.val[2]);

if(allianceColor.equalsIgnoreCase("red")) {

if (left_br > 90 && right_br > 90) pattern = 3;
else if (left_br > 90 && right_br < 90) pattern = 2;
else if (left_br < 90 && right_br > 90) pattern = 1;
else if (left_br < 90 && right_br < 90) {
    if (left_br > right_br) {
        pattern = 1;
    } else if (left_br < right_br) {
        pattern = 2;
    } else {
        pattern = 3;
    }
}
}

if (left_br > 95 && right_br > 95) pattern = 1;
else if (left_br > 95 && right_br < 95) pattern = 3;
else if (left_br < 95 && right_br > 95) pattern = 2;
else if (left_br < 95 && right_br < 95) {
    if (left_br > right_br) {
        pattern = 1;
    } else if (left_br < right_br) {
        pattern = 2;
    } else {
        pattern = 3;
    }
}
/*else {
if (left_br > 85 && right_br > 85) pattern = 3;
else if (left_br > 85 && right_br < 85) pattern = 1;
else if (left_br < 85 && right_br > 85) pattern = 2;
else if (left_br < 85 && right_br < 85) {
    if (left_br > right_br) {
        pattern = 1;
    } else if (left_br < right_br) {
        pattern = 2;
    } else {
        pattern = 3;
    }
}
}
*/
           /* return input;
        }

        private int get_hue(int red, int green, int blue) {

            float min = Math.min(Math.min(red, green), blue);
            float max = Math.max(Math.max(red, green), blue);

            if (min == max) {
                return 0;
            }

            float hue = 0f;
            if (max == red) {
                hue = (green - blue) / (max - min);

            } else if (max == green) {
                hue = 2f + (blue - red) / (max - min);

            } else {
                hue = 4f + (red - green) / (max - min);
            }

            hue = hue * 60;
            if (hue < 0) hue = hue + 360;

            return Math.round(hue);
        }

        private int get_brightness(int red, int green, int blue) {
            return (int) (((double) (red + green + blue)) / 3);
        }

    }
*/

    }

    public int getPattern() {
        return pattern;
    }

    public Scalar getLeftMean() {
        return left_mean;
    }

    public Scalar getRight_mean() {
        return right_mean;
    }
}
