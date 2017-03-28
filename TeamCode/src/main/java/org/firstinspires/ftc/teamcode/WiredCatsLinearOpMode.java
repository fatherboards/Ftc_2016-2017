package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by zipper on 2/27/17.
 */

public abstract class WiredCatsLinearOpMode extends LinearOpMode {
    //Drive Train
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    //Shooter
    DcMotor intake;
    DcMotor shooter;

    //Winch
    DcMotor winchleft;
    DcMotor winchright;

    //Side Pushers
    Servo leftButtonPusher;
    Servo rightButtonPusher;

    //Release
    Servo release;

    //FrontPusher
    Servo leftFrontPusher;
    Servo rightFrontPusher;

    //Gate
    Servo shooterGate;


    //Counters
    private ElapsedTime runtime = new ElapsedTime();

    //Sensors
    ColorSensor beaconColorLeft;
    ColorSensor beaconColorRight;

    /**
     * Initializes all necessary components including
     * Motors
     * Sensors
     * Servos
     */
    public void initialize(){
        beaconColorLeft = hardwareMap.colorSensor.get("beaconColorLeft");
        beaconColorRight = hardwareMap.colorSensor.get("beaconColorRight");
        leftFront = hardwareMap.dcMotor.get("fl");
        rightFront = hardwareMap.dcMotor.get("fr");
        leftBack = hardwareMap.dcMotor.get("bl");
        rightBack = hardwareMap.dcMotor.get("br");

        intake = hardwareMap.dcMotor.get("int");
        shooter = hardwareMap.dcMotor.get("s");

        winchleft = hardwareMap.dcMotor.get("wl");
        winchright = hardwareMap.dcMotor.get("wr");

        leftButtonPusher = hardwareMap.servo.get("leftButtonPusher");
        rightButtonPusher = hardwareMap.servo.get("rightButtonPusher");

        release = hardwareMap.servo.get("capDropper");

        shooterGate = hardwareMap.servo.get("shooterGate");

        leftFrontPusher = hardwareMap.servo.get("leftFrontPusher");
        rightFrontPusher = hardwareMap.servo.get("rightFrontPusher");

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        winchright.setDirection(DcMotorSimple.Direction.REVERSE);

        beaconColorRight.setI2cAddress(I2cAddr.create8bit(0x5c));

        leftButtonPusher.setPosition(0);
        rightButtonPusher.setPosition(1);
        release.setPosition(0.5);
        leftFrontPusher.setPosition(0);
        rightFrontPusher.setPosition(1);
        shooterGate.setPosition(0);

        telemetry.addData("Initialization ", "complete");
        telemetry.update();
    }

    /**
     * Get the color the colorsensor sees
     * @param colorSensor
     * color sensor to read
     * @return
     * Returns a string of the color seen, blue, red, or other
     * @throws InterruptedException
     */
    public String getColor(ColorSensor colorSensor) throws InterruptedException {
        if(colorSensor.blue() > colorSensor.red()+2) {
            return "blue";
        }
        else if(colorSensor.red() > colorSensor.blue()+2) {
            return "red";
        }
        else {
            return "other";
        }
    }

    /**
     * Telemetries all debugging information
     * @throws InterruptedException
     */
    public void telemetry() throws InterruptedException {
        telemetry.addData("colorLeft", getColor(beaconColorLeft));
        telemetry.addData("colorRight", getColor(beaconColorRight));
        telemetry.addData("leftFront to position",(leftFront.getTargetPosition() - leftFront.getCurrentPosition()));
        telemetry.addData("rightFront to position",(rightFront.getTargetPosition() - rightFront.getCurrentPosition()));
        telemetry.addData("leftBack to position",(leftBack.getTargetPosition() - leftBack.getCurrentPosition()));
        telemetry.addData("rightBack to position",(rightBack.getTargetPosition() - rightBack.getCurrentPosition()));
        telemetry.update();
    }

    /**
     * Zeroes encoders and resets them
     */
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

    /**
     * Stops all motors
     * @throws InterruptedException
     */
    public void stopAll() throws InterruptedException{
        telemetry();
        setPower(0);
        shooter.setPower(0);
        intake.setPower(0);
    }

    /**
     * Drive based on encoder ticks
     * @param leftSpeed
     * Speed for left motors
     * @param rightSpeed
     * Speed for right motors
     * @param leftCounts
     * Ticks for left encoders
     * @param rightCounts
     * Ticks for right encoders
     * @param timeoutS
     * Timeout seconds
     * @throws InterruptedException
     */
    public void encoderDrive(double leftSpeed, double rightSpeed, double leftCounts, double rightCounts, double timeoutS) throws InterruptedException {
        zeroEncoders();

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // reset the timeout time and start motion.
            runtime.reset();
            setPower(leftSpeed,rightSpeed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() && (runtime.seconds() < timeoutS)) {
                telemetry();
//                if(Math.abs(leftFront.getCurrentPosition()) > Math.abs(leftCounts)*(7.0/8) || Math.abs(leftBack.getCurrentPosition()) > Math.abs(leftCounts)*(7.0/8) || Math.abs(rightFront.getCurrentPosition()) > Math.abs(rightCounts)*(7.0/8) || Math.abs(rightBack.getCurrentPosition()) > Math.abs(rightCounts)*(7.0/8)) {
//                    leftFront.setPower(.5*leftSpeed);
//                    rightFront.setPower(.5*rightSpeed);
//                    leftBack.setPower(.5*leftSpeed);
//                    rightBack.setPower(.5*rightSpeed);
//                    telemetry.addData("speed",.5);
//                }
//                else if(Math.abs(leftFront.getCurrentPosition()) > Math.abs(leftCounts)*(.5) || Math.abs(leftBack.getCurrentPosition()) > Math.abs(leftCounts)*(.5) || Math.abs(rightFront.getCurrentPosition()) > Math.abs(rightCounts)*(.5) || Math.abs(rightBack.getCurrentPosition()) > Math.abs(rightCounts)*(.5)) {
//                    leftFront.setPower(.75*leftSpeed);
//                    rightFront.setPower(.75*rightSpeed);
//                    leftBack.setPower(.75*leftSpeed);
//                    rightBack.setPower(.75*rightSpeed);
//                    telemetry.addData("speed",.75);
//                }
//                else {
//                    telemetry.addData("speed",1);
//                }
                if(Math.abs(leftFront.getCurrentPosition()) > Math.abs(leftCounts) || Math.abs(leftBack.getCurrentPosition()) > Math.abs(leftCounts) || Math.abs(rightFront.getCurrentPosition()) > Math.abs(rightCounts) || Math.abs(rightBack.getCurrentPosition()) > Math.abs(rightCounts)) {
                    stopAll();
                    break;
                }
                idle();
            }
            stopAll();

            zeroEncoders();
            //sleep(250);   // optional pause after each move
        }
    }
    public void setPower(double power) {
        leftFront.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(power);
        rightBack.setPower(power);
    }
    public void setPower(double leftPower,double rightPower) {
        leftFront.setPower(leftPower);
        leftBack.setPower(leftPower);
        rightFront.setPower(rightPower);
        rightBack.setPower(rightPower);
    }
    public void forwardTicks(double power, int ticks, double timeout) throws InterruptedException {
        encoderDrive(power,power,ticks,ticks,timeout);
    }
    public void backwardTicks(double power, int ticks, double timeout) throws InterruptedException {
        encoderDrive(-power,-power,ticks,ticks,timeout);
    }
    public void rightTicks(double power, int ticks, double timeout) throws InterruptedException {
        encoderDrive(power,-power,ticks,ticks,timeout);
    }
    public void leftTicks(double power, int ticks, double timeout) throws InterruptedException {
        encoderDrive(-power,power,ticks,ticks,timeout);
    }
    public void forward(double power) {
        setPower(power);
    }
    public void backward(double power) {
        setPower(-power);
    }

    public void intake(String team,boolean doIt) throws InterruptedException {
        /*if(doIt) {
            if(!getColor(ballRejection).equals(team)) {
                intake.setPower(-1);
                sleep(1000);
                intake.setPower(0);
            }
            else {
                intake.setPower(1);
            }
        }
        else {
            intake.setPower(0);
        }*/
    }
    /*public String getBeaconColor() throws InterruptedException {
        return getColor(beaconColor);
    }
    public void enableShootingMode() {
        shooter.setPower(1);
        sleep(2000);
        intake.
        shooterGate.setPower(1);
    }
    public void disableShootingMode() {
        shooter.setPower(0);
        shooterGate.setPower(0);
    }
    public void slapBeaconLeft() {
        beaconRollerLeft.setPower(1);
    }
    public void unSlapBeaconLeft() {
        beaconRollerLeft.setPower(0);
    }
    public void slapBeaconRight() {
        beaconRollerLeft.setPower(1);
    }
    public void unSlapBeaconRight() {
        beaconRollerLeft.setPower(0);
    }*/
}
