package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by wross on 12/16/16.
 */
@Autonomous(name="One Ball Center", group = "Ball Autonomous")
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