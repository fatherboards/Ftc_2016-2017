package org.firstinspires.ftc.teamcode;

/**
 * Created by wross on 12/16/16.
 *
 * Red 2 beacon auto. Also scores two balls in the beginning, and parks on the corner vortex in the end.
 */
//@Autonomous(name="Red 2 Beacon", group="Beacon Autonomous")
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
        encoderDrive(.4,.4,3500,3500,10);
        encoderDrive(-.75,.75,700,700,3);
//        imuLeft(70,.35);
        idle();
        sleep(100);
        encoderDrive(.18,.18,234234,234234,1.5);
        encoderDrive(-.19,-.19,170,170,3);
        autoBeaconSlider.setPower(0);

//        imuLeft(10,.35);
        encoderDrive(-.75,.75,500,500,3);
        balance();
        autoBeaconSlider.setPower(getPowerDist());
        sleep(500);
        doBeacon(true,"red");
        idle();
        balance();
        encoderDrive(.4,.4,1700,1700,5);
//        balance();
        doBeacon(false,"red");
        encoderDrive(-.5,.5,150,150,2);
        encoderDrive(.5,.5,5000,5000,5);

    }


}