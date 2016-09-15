package org.firstinspires.ftc.robotcontroller.external.samples;

import android.graphics.Bitmap;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
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
public class OpenCVTestOpMode extends OpMode {
    Mat curFrame = new Mat();


    @Override
    public void init() {
    }

    @Override
    public void loop() {
        if(FtcRobotControllerActivity.curFrame!=null) {
            curFrame = FtcRobotControllerActivity.curFrame;
            Log.i("Loop", "PROCESSING");
            Mat grayed = new Mat();
            Mat canned = new Mat();
            Mat threshed = new Mat();
            ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();
            writeBitmap(curFrame, "opencvframe.png");
            Imgproc.cvtColor(curFrame, grayed, Imgproc.COLOR_BGR2GRAY);
            Imgproc.Canny(grayed, canned, 10, 10);
            Imgproc.threshold(canned, threshed, 127, 127, Imgproc.THRESH_BINARY);
            Imgproc.findContours(threshed, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(curFrame, contours, -1, new Scalar(0, 100, 100), 3);
            writeBitmap(curFrame, "opencvtestbmp.png");
            stop();
        }

    }

    public void writeBitmap(Mat cur,String filename) {
        Bitmap save = Bitmap.createBitmap(curFrame.width(),curFrame.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(cur,save);
        FileOutputStream out = null;
        try {
            out = new FileOutputStream(filename);
            save.compress(Bitmap.CompressFormat.PNG, 100, out); // bmp is your Bitmap instance
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
