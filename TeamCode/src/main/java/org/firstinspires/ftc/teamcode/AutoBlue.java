package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
/**
 * Created by wross on 12/16/16.
 *
 * Blue 2 beacon auto. Also scores two balls in the beginning, and parks on the corner vortex in the end.
 */
@Autonomous(name = "Blue 2 Beacon", group="Beacon Autonomous")
public class AutoBlue extends FatherboardsLinearOpMode{

    public void runOpMode() throws InterruptedException{
        initialize();
        while (!(isStarted() || isStopRequested())) {
            telemetry();
            idle();
        }
        waitForStart();
        encoderDrive(.3,.3,1650,1650,5);
        shootBall();
        sleep(500);
        intake();
        sleep(100);
        idle();
        shootBall();
        encoderDrive(.75,-.75,500,500,3);
        stopAll();
        encoderDrive(.4,.4,3550,3550,10);
        encoderDrive(.18,.18,900,900,1.25);
        encoderDrive(-.19,-.19,100,100,3);
        encoderDrive(-.75,.75,500,500,3);
        autoBeaconSlider.setPower(0);
        idle();
        balance();
        autoBeaconSlider.setPower(getPowerDist());
        sleep(500);
        doBeacon(false,"blue");
        idle();
//        balance();
        autoBeaconSlider.setPower(0);
        encoderDrive(-.4,-.4,1700,1700,5);
//        balance();
        doBeacon(true,"blue");
        encoderDrive(.5,-.5,150,150,2);
        encoderDrive(-.5,-.5,5000,5000,5);

    }


}