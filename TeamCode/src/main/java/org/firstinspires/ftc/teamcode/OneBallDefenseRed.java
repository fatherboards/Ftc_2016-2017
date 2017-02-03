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
@Autonomous(name="One Ball Defense Red", group = "Ball Autonomous")
public class OneBallDefenseRed extends LinearOpMode{
    private DcMotor leftBack;
    private DcMotor leftFront;
    private DcMotor pickpMechanism;
    private DcMotor rightBack;
    private DcMotor rightFront;
    private DcMotor shooter;
    private CRServo autoBeaconSlider;
    private CRServo slider;
    private OpticalDistanceSensor ballCheck;
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime mainTime = new ElapsedTime();
    public void initialize(){
        shooter = hardwareMap.dcMotor.get("shooter");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        autoBeaconSlider = hardwareMap.crservo.get("slider");
        pickpMechanism = hardwareMap.dcMotor.get("pickupMechanism");
        slider = hardwareMap.crservo.get("autoBeaconSlider");
        ballCheck = hardwareMap.opticalDistanceSensor.get("ballODS");
        autoBeaconSlider.setPower(-.7);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.REVERSE);
        pickpMechanism.setDirection(DcMotor.Direction.REVERSE);
    }
    public void encoderDrive(double leftSpeed, double rightSpeed, double leftCounts, double rightCounts, double timeoutS) throws InterruptedException {
        zeroEncoders();

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // reset the timeout time and start motion.
            runtime.reset();
            leftFront.setPower(leftSpeed);
            rightFront.setPower(rightSpeed);
            leftBack.setPower(leftSpeed);
            rightBack.setPower(rightSpeed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() && (runtime.seconds() < timeoutS)) {
                telemetry();
                if(Math.abs(leftFront.getCurrentPosition()) > Math.abs(leftCounts) || Math.abs(leftBack.getCurrentPosition()) > Math.abs(leftCounts) || Math.abs(rightFront.getCurrentPosition()) > Math.abs(rightCounts) || Math.abs(rightBack.getCurrentPosition()) > Math.abs(rightCounts)) {
                    leftFront.setPower(0);
                    leftBack.setPower(0);
                    rightFront.setPower(0);
                    rightBack.setPower(0);
                    break;
                }
                idle();
            }
            stopAll();

            zeroEncoders();
            //sleep(250);   // optional pause after each move
        }
    }
    public void zeroEncoders() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void stopAll() throws InterruptedException{
        telemetry();
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        shooter.setPower(0);
        pickpMechanism.setPower(0);
    }
    public void telemetry() throws InterruptedException{
        telemetry.addData("leftFront to position",(leftFront.getTargetPosition() - leftFront.getCurrentPosition()));
        telemetry.addData("rightFront to position",(rightFront.getTargetPosition() - rightFront.getCurrentPosition()));
        telemetry.addData("leftBack to position",(leftBack.getTargetPosition() - leftBack.getCurrentPosition()));
        telemetry.addData("rightBack to position",(rightBack.getTargetPosition() - rightBack.getCurrentPosition()));
        telemetry.addData("ods", ballCheck.getLightDetected());
        telemetry.update();
    }
    public void shootBall() throws InterruptedException{
        shooter.setPower(.75);
        sleep(1000);
        shooter.setPower(0);
    }
    public boolean isIn(){
        if(ballCheck.getLightDetected() > .009){
            return true;
        }
        return false;
    }
    public boolean intake() throws InterruptedException{
        pickpMechanism.setPower(1);
        while(ballCheck.getLightDetected() < .014 && opModeIsActive()){
            telemetry();
        }
        pickpMechanism.setPower(0);
        return true;
    }

    public void runOpMode() throws InterruptedException{
        initialize();
        while (!(isStarted() || isStopRequested())) {
            telemetry();
            idle();
        }
        mainTime.reset();
        encoderDrive(.3, .3, 2205, 2250, 10);
        shootBall();
        encoderDrive(.3, .3, 5000, 5000, 10);
        encoderDrive(.75,-.75,1200,1200,5);
        while(opModeIsActive() && mainTime.seconds()<11) {
            idle();
        }
        encoderDrive(.3,.3,1500,1500,5);
        encoderDrive(.75,-.75,500,500,5);
        encoderDrive(.16,.16,1500,1500,5);
    }


}