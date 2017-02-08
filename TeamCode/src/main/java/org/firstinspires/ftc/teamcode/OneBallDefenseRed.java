package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
/**
 * Created by wross on 12/16/16.
 */
@Autonomous(name="One Ball Defense Red", group = "Ball Autonomous")
public class OneBallDefenseRed extends FatherboardsLinearOpMode{
    private ElapsedTime mainTime = new ElapsedTime();
    public void runOpMode() throws InterruptedException{
        initialize();
        while (!(isStarted() || isStopRequested())) {
            telemetry();
            idle();
        }
        mainTime.reset();
        encoderDrive(.4, .4, 2400, 2400, 10);
        shootBall();
        encoderDrive(.4, .4, 3500, 3500, 10);
        encoderDrive(.75,-.75,700,700,5);
        while(opModeIsActive() && mainTime.seconds()<11) {
            idle();
        }
        encoderDrive(.4, .4, 1500, 1500, 10);
        encoderDrive(.75,-.75,1000,1000,5);
        encoderDrive(.4,.4,1350,1350,5);
        encoderDrive(-.75,.75,1000,1000,5);
        encoderDrive(.15,.15,234234234,234234234,5);
    }


}