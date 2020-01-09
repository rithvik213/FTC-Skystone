package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

import static org.opencv.core.CvType.CV_8UC1;

@Autonomous
public class CbCameraTest extends LinearOpMode {

    OpenCvCamera phoneCamera;

    SamplePipeline stone_pipeline;

    String allianceColor = "blue";

    int pattern = 0;

    Scalar left_mean;
    Scalar right_mean;



    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
       // phoneCamera = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCamera.openCameraDevice();

        stone_pipeline = new SamplePipeline();
        phoneCamera.setPipeline(stone_pipeline);

        phoneCamera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        waitForStart();

        while(opModeIsActive()) {

            telemetry.addData("Pattern: ", pattern);
            telemetry.addData("Left_mean", getLeftMean().val[0]);
            telemetry.addData("Right_mean", getRight_mean().val[0]);
            telemetry.update();
        }

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
                        (int) (input.cols() * (6f / 32f)),
                        (int) (input.rows() * (14f / 32f)),
                        (int) (input.cols() * (14f / 32f)), //previously 11 with 17
                        (int) (input.rows() * (18f / 32f))

                        /*sideways right config
                        (int) (input.cols() * (11f / 32f)),
                        (int) (input.rows() * (12f / 32f)),
                        (int) (input.cols() * (17f / 32f)),
                        (int) (input.rows() * (17f / 32f))
                         */
                };

                right_rect = new int[]{
                        (int) (input.cols() * (14f / 32f)),
                        (int) (input.rows() * (14f / 32f)),
                        (int) (input.cols() * (22f / 32f)), //previously 19 with 25
                        (int) (input.rows() * (18f / 32f))

                        /*sideways right config
                         (int) (input.cols() * (19f / 32f)),
                        (int) (input.rows() * (12f / 32f)),
                        (int) (input.cols() * (25f / 32f)),
                        (int) (input.rows() * (17f / 32f))
                         */
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
                         */
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
                         */
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

            left_mean = Core.mean(left_block);


            right_mean = Core.mean(right_block);

            if(allianceColor.equalsIgnoreCase("red")) {

                if(Math.abs(left_mean.val[0] - right_mean.val[0]) <= 25) pattern = 3;
                else if(left_mean.val[0] < right_mean.val[0]) pattern = 1;
                else if(left_mean.val[0] > right_mean.val[0]) pattern = 2;

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
            }

            else {

                if(Math.abs(left_mean.val[0] - right_mean.val[0]) <= 25) pattern = 3;
                else if(left_mean.val[0] < right_mean.val[0]) pattern = 2;
                else if(left_mean.val[0] > right_mean.val[0]) pattern = 1;

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

    }
    public Scalar getLeftMean() {
        return left_mean;
    }

    public Scalar getRight_mean() {
        return right_mean;
    }
}
