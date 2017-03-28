package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by STUDENT on 3/22/2017.
 */
//@Autonomous(name = "Blue Auto", group = "Autonomous")
public class autobloo extends LinearOpMode{

    //Drive Train
    DcMotor frontleft;
    DcMotor frontright;
    DcMotor backleft;
    DcMotor backright;

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

    //Color Sensors
    ColorSensor beaconColorLeft;
    ColorSensor beaconColorRight;

    //Counters
    int intakeToggle = 0;
    int outtakeToggle = 0;
    int shooterToggle = 0;
    int leftButtonPusherToggle = 0;
    int rightButtonPusherToggle = 0;
    int releaseToggle = 0;
    int frontPusherToggle = 0;
    int shooterGateToggle = 0;

    //Bang Bang Stuff
    private static final double TOLERANCE = 0.5e-7;
    private static final double TARGET_VELOCITY = 2.567e-6;

    protected static final double VELOCITY_P = 7.2;
    protected static final double VELOCITY_I = 0.1;
    protected static final double VELOCITY_D = 0.0;
    VelocityCalculator flywheelVelocity = new VelocityCalculator();
    BangBangCalculator velocityBangBang = new BangBangCalculator();

    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double COUNTS_PER_MOTOR_REV2 = 1120;
    static final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double COUNTS_PER_INCH2 = (COUNTS_PER_MOTOR_REV2 * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() throws InterruptedException {

        frontleft = hardwareMap.dcMotor.get("fl");
        frontright = hardwareMap.dcMotor.get("fr");
        backleft = hardwareMap.dcMotor.get("bl");
        backright = hardwareMap.dcMotor.get("br");

        intake = hardwareMap.dcMotor.get("int");
        shooter = hardwareMap.dcMotor.get("s");

        winchleft = hardwareMap.dcMotor.get("wl");
        winchright = hardwareMap.dcMotor.get("wr");

        leftButtonPusher = hardwareMap.servo.get("leftButtonPusher");
        rightButtonPusher = hardwareMap.servo.get("rightButtonPusher");

        release = hardwareMap.servo.get("capDropper");

        leftFrontPusher = hardwareMap.servo.get("leftFrontPusher");
        rightFrontPusher = hardwareMap.servo.get("rightFrontPusher");

        shooterGate = hardwareMap.servo.get("shooterGate");

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontright.setDirection(DcMotorSimple.Direction.REVERSE);
        backright.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        winchright.setDirection(DcMotorSimple.Direction.REVERSE);

        leftButtonPusher.setPosition(0);
        rightButtonPusher.setPosition(1);
        release.setPosition(0.5);
        leftFrontPusher.setPosition(0);
        rightFrontPusher.setPosition(1);
        shooterGate.setPosition(0);

        /*beaconColorLeft.setI2cAddress(I2cAddr.create8bit(0x5c));
        beaconColorRight.setI2cAddress(I2cAddr.create8bit(0x3c));*/

        telemetry.addData("Initialization ", "complete");
        telemetry.update();

        waitForStart();
        leftButtonPusher.setPosition(1.0);
        sleep(300);
        leftButtonPusher.setPosition(0.0);
        sleep(50);
        backwardTicks(0.3, 200, 5000);
        sleep(10);
        leftTicks(-0.5, 300, 5000);
        sleep(10);
        backwardTicks(0.3, 6000, 5000);
        sleep(10);
        leftTicks(0.3, 300, 5000);
        sleep(10);

    }
    /*public String getColor(ColorSensor colorSensor) throws InterruptedException {
        if(colorSensor.blue() > colorSensor.red()+2) {
            return "blue";
        }
        else if(colorSensor.red() > colorSensor.blue()+2) {
            return "red";
        }
        else {
            return "other";
        }
    }*/

    /**
     * Telemetries all debugging information
     * @throws InterruptedException
     */
    public void telemetry() throws InterruptedException {
       // telemetry.addData("colorLeft", getColor(beaconColorLeft));
       // telemetry.addData("colorRight", getColor(beaconColorRight));
        telemetry.addData("leftFront to position",(frontleft.getTargetPosition() - frontleft.getCurrentPosition()));
        telemetry.addData("rightFront to position",(frontright.getTargetPosition() - frontright.getCurrentPosition()));
        telemetry.addData("leftBack to position",(backleft.getTargetPosition() - backleft.getCurrentPosition()));
        telemetry.addData("rightBack to position",(backright.getTargetPosition() - backright.getCurrentPosition()));
        telemetry.update();
    }

    /**
     * Zeroes encoders and resets them
     */
    public void zeroEncoders() {
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
                if(Math.abs(frontleft.getCurrentPosition()) > Math.abs(leftCounts) || Math.abs(backleft.getCurrentPosition()) > Math.abs(leftCounts) || Math.abs(frontright.getCurrentPosition()) > Math.abs(rightCounts) || Math.abs(backright.getCurrentPosition()) > Math.abs(rightCounts)) {
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
        frontleft.setPower(power);
        backleft.setPower(power);
        frontright.setPower(power);
        backright.setPower(power);
    }
    public void setPower(double leftPower,double rightPower) {
        frontleft.setPower(leftPower);
        backleft.setPower(leftPower);
        frontright.setPower(rightPower);
        backright.setPower(rightPower);
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
