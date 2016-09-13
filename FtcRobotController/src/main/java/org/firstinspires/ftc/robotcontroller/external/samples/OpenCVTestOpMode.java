package org.firstinspires.ftc.robotcontroller.external.samples;

import android.graphics.Bitmap;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;

/**
 * Created by zipper on 9/12/16.
 */
@Autonomous(name="Opencv: Zipper's first opencv test", group="OpenCV")
@Disabled
public class OpenCVTestOpMode extends OpMode implements CameraBridgeViewBase.CvCameraViewListener2 {
    CameraBridgeViewBase mCameraBridgeViewBase;
    Mat curFrame;
    boolean processing;
    @Override
    public void init() {
        mCameraBridgeViewBase = (CameraBridgeViewBase) FtcRobotControllerActivity.activity.findViewById(R.id.javaCameraView);
        mCameraBridgeViewBase.setCvCameraViewListener(this);
    }

    @Override
    public void loop() {
        if(processing) {
            Mat canned = new Mat();
            Mat threshed = new Mat();
            ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();
            Bitmap save = Bitmap.createBitmap(curFrame.width(),curFrame.height(), Bitmap.Config.RGB_565);
            Imgproc.Canny(curFrame,canned,10,10);
            Imgproc.threshold(canned,threshed,127,127,Imgproc.THRESH_BINARY);
            Imgproc.findContours(threshed,contours,new Mat(), Imgproc.RETR_TREE,Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(curFrame,contours,-1,new Scalar(0,100,100),3);
            Utils.matToBitmap(curFrame,save);
            writeBitmap(save,"opencvtestbmp.png");
        }
    }


    @Override
    public void onCameraViewStarted(int width, int height) {
        curFrame = new Mat();
    }

    @Override
    public void onCameraViewStopped() {

    }

    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        processing = true;
        curFrame = inputFrame.rgba();
        return curFrame;
    }
    public void writeBitmap(Bitmap bmp,String filename) {
        FileOutputStream out = null;
        try {
            out = new FileOutputStream(filename);
            bmp.compress(Bitmap.CompressFormat.PNG, 100, out); // bmp is your Bitmap instance
            // PNG is a lossless format, the compression factor (100) is ignored
        } catch (Exception e) {
            e.printStackTrace();
        } finally {
            try {
                if (out != null) {
                    out.close();
                }
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }
}
