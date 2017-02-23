package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

/**
 * Created by wyatt.ross on 1/25/2017.
 */
@TeleOp
public class LinearTeleop extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();
    private boolean isShooting = false;
    private DcMotor leftBack;
    private DcMotor leftFront;
    private DcMotor pickpMechanism;
    private DcMotor rightBack;
    private DcMotor rightFront;
    private DcMotor shooter;
    private CRServo autoBeaconSlider;
    private CRServo slider;
    private OpticalDistanceSensor ballCheck;
    private double NORMALSPEED = 1.00;
    private double BEACONSPEED = .15;
    private double multiplier = NORMALSPEED;

    public String isIn(){
        if(ballCheck.getLightDetected() > .014){
            return "YEEEEEE :^)";
        }
        return "nah";
    }

    public void ShootMode()throws InterruptedException{
        if(!isInMacro()) {
            pickpMechanism.setPower(1);
        }
        else{
            sleep(200);
            pickpMechanism.setPower(0);
            shootBall();
        }
    }

    public void shootBall() throws InterruptedException{
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < .75) {
            sleep(75);
            shooter.setPower(.75);
            leftFront.setPower(gamepad1.left_stick_y*multiplier);
            leftBack.setPower(gamepad1.left_stick_y*multiplier);
            rightFront.setPower(gamepad1.right_stick_y*multiplier);
            rightBack.setPower(gamepad1.right_stick_y*multiplier);
            idle();
        }
        shooter.setPower(0);
    }

    public boolean isInMacro(){
        return ballCheck.getLightDetected() > .015;
    }

    public boolean intake() throws InterruptedException{
        pickpMechanism.setPower(1);
        runtime.reset();
        while(!isInMacro() && runtime.seconds() < 5 && opModeIsActive()) {
            leftFront.setPower(gamepad1.left_stick_y*multiplier);
            leftBack.setPower(gamepad1.left_stick_y*multiplier);
            rightFront.setPower(gamepad1.right_stick_y*multiplier);
            rightBack.setPower(gamepad1.right_stick_y*multiplier);
            idle();
        }
        pickpMechanism.setPower(0);
        return true;
    }

    @Override
    public void runOpMode() throws InterruptedException{
        shooter = hardwareMap.dcMotor.get("shooter");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        autoBeaconSlider = hardwareMap.crservo.get("autoBeaconSlider");
        pickpMechanism = hardwareMap.dcMotor.get("pickupMechanism");
        slider = hardwareMap.crservo.get("slider");
        ballCheck = hardwareMap.opticalDistanceSensor.get("ballODS");
        autoBeaconSlider.setPower(-.95);
        slider.setPower(-.5);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.REVERSE);
//        pickpMechanism.setDirection(DcMotor.Direction.REVERSE);
        while(!isStarted()){
            idle();
        }

        while(opModeIsActive()){
            telemetry.addData("is loaded: ", isIn());
            telemetry.addData("Light: ", ballCheck.getLightDetected());
            telemetry.addData("isShooting: ", isShooting);
            telemetry.update();
            leftFront.setPower(gamepad1.left_stick_y*multiplier);
            leftBack.setPower(gamepad1.left_stick_y*multiplier);
            rightFront.setPower(gamepad1.right_stick_y*multiplier);
            rightBack.setPower(gamepad1.right_stick_y*multiplier);
            if(gamepad2.left_bumper){
                autoBeaconSlider.setPower(-1);
            }
            else if(gamepad2.right_bumper){
                autoBeaconSlider.setPower(1);
            }
            else{
                autoBeaconSlider.setPower(.08);
            }
            if(gamepad2.b){
                pickpMechanism.setPower(1);
            }
            else if(gamepad2.a){
                pickpMechanism.setPower(-1);
            }
            else if(gamepad2.x){
                pickpMechanism.setPower(0);
            }
            if(gamepad1.left_bumper){
                isShooting = true;
            }
            if(gamepad1.right_bumper){
                isShooting = false;
                pickpMechanism.setPower(0);
            }
            if(gamepad1.left_trigger > .1){
                shooter.setPower(.75);
            }
            else{
                shooter.setPower(0);
            }
            if(gamepad1.a){
                multiplier = NORMALSPEED;
            }
            if(gamepad1.b){
                multiplier = BEACONSPEED;
            }
            if(isShooting) {
                ShootMode();
            }



        }
        idle();
    }
}