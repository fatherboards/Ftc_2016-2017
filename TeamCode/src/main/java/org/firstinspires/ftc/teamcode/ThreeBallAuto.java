package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Created by wross on 12/16/16.
 */
//@Autonomous(name="Three Ball Center", group = "Ball Autonomous")
public class ThreeBallAuto extends FatherboardsLinearOpMode{
    private ElapsedTime myRuntime = new ElapsedTime();
    public void runOpMode() throws InterruptedException{
        initialize();
        while (!(isStarted() || isStopRequested())) {
            telemetry();
            idle();
        }
        myRuntime.reset();
        encoderDrive(.3, .3, 2100, 2100, 10);
        intake();
        shootBall();
        intake();
        shootBall();
        intake();
        shootBall();
        encoderDrive(.3, .3, 1200, 1200, 10);
    }


}