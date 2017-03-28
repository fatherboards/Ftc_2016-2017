package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Created by wross on 12/16/16.
 *
 * Auto opmode that scores one ball in autonomous, then hits the ball and parks in the last 4 seconds.
 */
//@Autonomous(name="One Ball Center", group = "Ball Autonomous")
public class OneBallAuto extends FatherboardsLinearOpMode{
    ElapsedTime autoRunTime = new ElapsedTime();
    public void runOpMode() throws InterruptedException{
        initialize();
        while (!(isStarted() || isStopRequested())) {
            telemetry();
            idle();
        }
        autoRunTime.reset();
        encoderDrive(.3, .3, 2100, 2100, 10);
        shootBall();
        while(autoRunTime.seconds() < 26) {
            if(isIn()) {
                shootBall();
            }
        }
        encoderDrive(.3, .3, 1200, 1200, 10);
    }


}