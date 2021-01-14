package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name = "stackDetector", group = "Teleop")
public class stackDetector extends OpMode {

    OpenCvCamera phoneCam = null;
    static double ringCount = 0;
    int rect1X = 0;
    int rect1Y = 0;

    int rect2X = 0;
    int rect2Y = 0;

    @Override
    public void init() {

    }

    @Override
    public void init_loop()
    {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        phoneCam.setPipeline(new ringDetectingPipeline());

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {

                phoneCam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);

            }
        });

    }


    @Override
    public void loop()
    {
        if (ringCount == 4){

        }
        if (ringCount == 1){

        }
        if (ringCount == 0){

        }
    }

    class ringDetectingPipeline extends OpenCvPipeline{

        Mat YCbCr = new Mat();
        Mat outPut = new Mat();
        Mat upperCrop = new Mat();
        Mat lowerCrop = new Mat();


        @Override
        public Mat processFrame(Mat input) {

            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);

            input.copyTo(outPut);

            Rect rect1 = new Rect(rect1X, rect1Y, 119, 69);
            Rect rect2 = new Rect(rect2X, rect2Y, 119, 20);

            Scalar rectangleColor = new Scalar(255,0,0);

            Imgproc.rectangle(outPut, rect1, rectangleColor, 2);
            Imgproc.rectangle(outPut, rect2, rectangleColor, 2);

            lowerCrop = YCbCr.submat(rect1);

            upperCrop = YCbCr.submat(rect2);

            Core.extractChannel(lowerCrop, lowerCrop, 2);
            Core.extractChannel(upperCrop, upperCrop, 2);

            Scalar lowerAverageOrange = Core.mean(lowerCrop);
            Scalar upperAverageOrange = Core.mean(upperCrop);

            double finalLowerAverage = lowerAverageOrange.val[0];
            double finalUpperAverage = upperAverageOrange.val[0];

            telemetry.addData("Lower Average", finalLowerAverage);
            telemetry.addData("Upper Average", finalUpperAverage);

            if (finalLowerAverage > 15 && finalLowerAverage < 130 && finalUpperAverage < 130){

                ringCount = 4.0;

            }else if(finalLowerAverage > 10 && finalUpperAverage < 15 && finalLowerAverage > 10 && finalUpperAverage < 15){
                ringCount = 0.0;
            }else{
                ringCount = 1.0;
            }



            return outPut;
        }
    }



}
