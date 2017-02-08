package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
/**
 * Created by wross on 12/16/16.
 */
@Autonomous(name="Red 2 Beacon", group="Beacon Autonomous")
public class AutoRed extends FatherboardsLinearOpMode{


    public void runOpMode() throws InterruptedException{
        initialize();
        while (!(isStarted() || isStopRequested())) {
            telemetry();
            idle();
        }
        waitForStart();
        encoderDrive(.4,.4,1500,1500,5);
        sleep(250);
        idle();
        shootBall();
        idle();
        sleep(400);
        intake();
        sleep(200);
        idle();
        shootBall();
//        imuLeft(40,.35);
        encoderDrive(-.75,.75,400,400,3);
        stopAll();
        encoderDrive(.4,.4,3800,3800,10);
        encoderDrive(-.75,.75,1100,1100,3);
//        imuLeft(70,.35);
        idle();
        sleep(100);
        encoderDrive(.14,.14,234234,234234,1.5);
        encoderDrive(-.19,-.19,100,100,3);
        autoBeaconSlider.setPower(0);

//        imuLeft(10,.35);
        encoderDrive(-.75,.75,500,500,3);
        autoBeaconSlider.setPower(getPowerDist());
        sleep(500);
        idle();
        balance();
        autoBeaconSlider.setPower(getPowerDist());
//        sleep(500);
        doBeacon(true,"red");
        idle();
        balance();
        autoBeaconSlider.setPower(getPowerDist());
        encoderDrive(.4,.4,1700,1700,5);
//        balance();
        doBeacon(false,"red");
        encoderDrive(-.5,.5,150,150,2);
        encoderDrive(.5,.5,5000,5000,5);

    }


}