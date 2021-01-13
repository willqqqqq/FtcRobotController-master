package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

public class stackDetector extends OpMode {

    OpenCvCamera phoneCam = null;

    static double ringCount = 0;

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

    }

    class ringDetectingPipeline extends OpenCvPipeline{

        Mat YCbCr = new Mat();

        @Override
        public Mat processFrame(Mat input) {


            return null;
        }
    }



}
