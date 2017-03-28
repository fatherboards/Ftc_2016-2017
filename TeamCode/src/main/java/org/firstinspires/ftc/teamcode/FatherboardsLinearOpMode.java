package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by zipper on 2/4/17.
 *
 * Abstract class that contains the bulk of our code for the velocity vortex challenge.
 * All of the helper methods for our auto opmodes and teleop opmodes are neatly kept here.
 */

public abstract class FatherboardsLinearOpMode extends LinearOpMode {

    private DcMotor leftBack;
    private DcMotor leftFront;
    private DcMotor pickpMechanism;
    private DcMotor rightBack;
    private DcMotor rightFront;
    private DcMotor shooter;
    protected CRServo autoBeaconSlider;
    private CRServo slider;
    private OpticalDistanceSensor ballCheck;
    private ElapsedTime runtime = new ElapsedTime();
    private ColorSensor color;

    private FatherboardsRangeSensor rangeSensorFront;
    private FatherboardsRangeSensor rangeSensorBack;
    private I2cDeviceSynch rangeSensorReaderBack;
    private I2cDevice rangeSensorBackDevice;

    private I2cDeviceSynch rangeSensorReaderFront;
    private I2cDevice rangeSensorFrontDevice;
    private MasqAdafruitIMU imu;

    /**
     * Initializes all necessary components including
     * Motors
     * Sensors
     * Servos
     */
    public void initialize(){
        color = hardwareMap.colorSensor.get("color");
        shooter = hardwareMap.dcMotor.get("shooter");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        autoBeaconSlider = hardwareMap.crservo.get("slider");
        pickpMechanism = hardwareMap.dcMotor.get("pickupMechanism");
        slider = hardwareMap.crservo.get("autoBeaconSlider");
        ballCheck = hardwareMap.opticalDistanceSensor.get("ballODS");
        autoBeaconSlider.setPower(0.1);


        rightBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.REVERSE);

        color = hardwareMap.colorSensor.get("color");
        color.setI2cAddress(I2cAddr.create8bit(0x5a));

        rangeSensorBackDevice = hardwareMap.i2cDevice.get("rangeSensorBack");
        rangeSensorReaderBack = new I2cDeviceSynchImpl(rangeSensorBackDevice, I2cAddr.create8bit(0x38), false);
        rangeSensorFrontDevice = hardwareMap.i2cDevice.get("rangeSensorFront");
        rangeSensorReaderFront = new I2cDeviceSynchImpl(rangeSensorFrontDevice, I2cAddr.create8bit(0x28), false);
        rangeSensorBack = new FatherboardsRangeSensor(rangeSensorReaderBack,I2cAddr.create8bit(0x38));
        rangeSensorFront = new FatherboardsRangeSensor(rangeSensorReaderFront, I2cAddr.create8bit(0x28));
        telemetry.addData("Initialization ", "complete");
        telemetry.update();
    }

    /**
     * Turn right using the IMU
     * @param degs
     * Turn this amount of degrees
     * @param speed
     * Turn at this speed
     */
    public void imuRight(int degs, double speed){
        double[] angles = imu.getAngles();
        double yaw = angles[0];
        double yawStart = (yaw+180)%360;
        // this adds telemetry data using the telemetrize() method in the MasqAdafruitIMU class
        while(Math.abs(yaw - ((yawStart - degs*.8) % 360)) > 3.5  && opModeIsActive()) {
            telemetry();
            telemetry.addData("Target",((yawStart - degs) % 360));
            telemetry.addData("Progress",Math.abs(yaw - ((yawStart - degs) % 360)));
            telemetry.update();
            angles = imu.getAngles();
            yaw = (angles[0]+180)%360;
            leftFront.setPower(speed);
            rightFront.setPower(-speed);
            leftBack.setPower(speed);
            rightBack.setPower(-speed);
        }
    }

    /**
     * Turn left using the IMU
     * @param degs
     * Turn this amount of degrees
     * @param speed
     * Turn at this speed
     */
    public void imuLeft(int degs, double speed){
        double[] angles = imu.getAngles();
        double yaw = angles[0];
        double yawStart = (yaw+180)%360;
        // this adds telemetry data using the telemetrize() method in the MasqAdafruitIMU class
        while(Math.abs(yaw - ((yawStart + degs*.8) % 360)) > 3.5  && opModeIsActive()) {
            telemetry();
            telemetry.addData("Target",((yawStart - degs) % 360));
            telemetry.addData("Progress",Math.abs(yaw - ((yawStart - degs) % 360)));
            telemetry.update();
            angles = imu.getAngles();
            yaw = (angles[0]+180)%360;
            leftFront.setPower(-speed);
            rightFront.setPower(speed);
            leftBack.setPower(-speed);
            rightBack.setPower(speed);
        }
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
     * Turn left indefinitely
     * @param power
     * Turn at this power
     * @throws InterruptedException
     */
    public void left(double power) throws InterruptedException {
        telemetry();
        leftFront.setPower(-power);
        leftBack.setPower(-power);
        rightFront.setPower(power);
        rightBack.setPower(power);
    }

    /**
     * Turn right indefinitely
     * @param power
     * Turn at this power
     * @throws InterruptedException
     */
    public void right(double power) throws InterruptedException {
        telemetry();
        leftFront.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(-power);
        rightBack.setPower(-power);
    }

    /**
     * This function uses two range sensors to make our robot parallel to the wall.
     * To  do so, it takes the distances from the wall on both sensors, and compensates by turning until the distances are the same.
     * One special part of this function is that we filter out bad values.
     * An inherent error of having two range sensors, is that the ultrasonic pulses from one sensor will interfere with the readings of the other sensor at times.
     * With that being said, with a lot of work, we were able to filter out the garbage value the sensor would return on a bad read, and have a very robust function.
     * @throws InterruptedException
     */
    public void balance() throws InterruptedException {
        runtime.reset();
        double cmBack = rangeSensorBack.getDistance(DistanceUnit.CM);
        double cmFront = rangeSensorFront.getDistance(DistanceUnit.CM);
        while (Math.abs(cmBack - cmFront) > 2 && opModeIsActive() && runtime.seconds() < 5){
            cmBack = rangeSensorBack.getDistance(DistanceUnit.CM);
            cmFront = rangeSensorFront.getDistance(DistanceUnit.CM);
            if(cmBack==255 || cmFront ==255) {
                stopAll();
                idle();
                continue;
            }
            if (cmFront > cmBack) {
                right(.24);
            }
            if (cmBack > cmFront) {
                left(.24);
            }
            telemetry();
            idle();
        }
        stopAll();
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
            leftFront.setPower(leftSpeed);
            rightFront.setPower(rightSpeed);
            leftBack.setPower(leftSpeed);
            rightBack.setPower(rightSpeed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() && (runtime.seconds() < timeoutS)) {
                telemetry();
                if(Math.abs(leftFront.getCurrentPosition()) > Math.abs(leftCounts)*(7.0/8) || Math.abs(leftBack.getCurrentPosition()) > Math.abs(leftCounts)*(7.0/8) || Math.abs(rightFront.getCurrentPosition()) > Math.abs(rightCounts)*(7.0/8) || Math.abs(rightBack.getCurrentPosition()) > Math.abs(rightCounts)*(7.0/8)) {
                    leftFront.setPower(.5*leftSpeed);
                    rightFront.setPower(.5*rightSpeed);
                    leftBack.setPower(.5*leftSpeed);
                    rightBack.setPower(.5*rightSpeed);
                    telemetry.addData("speed",.5);
                }
                else if(Math.abs(leftFront.getCurrentPosition()) > Math.abs(leftCounts)*(.5) || Math.abs(leftBack.getCurrentPosition()) > Math.abs(leftCounts)*(.5) || Math.abs(rightFront.getCurrentPosition()) > Math.abs(rightCounts)*(.5) || Math.abs(rightBack.getCurrentPosition()) > Math.abs(rightCounts)*(.5)) {
                    leftFront.setPower(.75*leftSpeed);
                    rightFront.setPower(.75*rightSpeed);
                    leftBack.setPower(.75*leftSpeed);
                    rightBack.setPower(.75*rightSpeed);
                    telemetry.addData("speed",.75);
                }
                else {
                    telemetry.addData("speed",1);
                }
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
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        shooter.setPower(0);
        pickpMechanism.setPower(0);
    }

    /**
     * Telemetries all debugging information
     */
    public void telemetry(){

        try {
            telemetry.addData("color", getColor(color));
        }catch(InterruptedException e){}
        telemetry.addData("Range Sensor Front", "%.2f cm", rangeSensorFront.getDistance(DistanceUnit.CM));
        telemetry.addData("Range Sensor Back", "%.2f cm", rangeSensorBack.getDistance(DistanceUnit.CM));
        telemetry.addData("leftFront to position",(leftFront.getTargetPosition() - leftFront.getCurrentPosition()));
        telemetry.addData("rightFront to position",(rightFront.getTargetPosition() - rightFront.getCurrentPosition()));
        telemetry.addData("leftBack to position",(leftBack.getTargetPosition() - leftBack.getCurrentPosition()));
        telemetry.addData("rightBack to position",(rightBack.getTargetPosition() - rightBack.getCurrentPosition()));
        telemetry.addData("ods", ballCheck.getLightDetected());
        telemetry.update();
    }

    /**
     * Shoots one ball
     * @throws InterruptedException
     */
    public void shootBall() throws InterruptedException{
        shooter.setPower(1);
        sleep(750);
        shooter.setPower(0);
    }

    /**
     * Helps us tell when a ball is in the launcher
     * @return
     * Returns whether the ball is in or not
     */
    public boolean isIn(){
        return ballCheck.getLightDetected() > .015;
    }

    /**
     * Intake until the ball is in
     * @throws InterruptedException
     *
     */
    public boolean intake() throws InterruptedException{
        pickpMechanism.setPower(1);
        runtime.reset();
        while(!isIn() && runtime.seconds() < 5 && opModeIsActive()) idle();
        pickpMechanism.setPower(0);
        return true;
    }

    /**
     * Go forward indefinitely
     * @param power
     * Drive at this power
     * @throws InterruptedException
     */
    public void forward(double power) throws InterruptedException{
        telemetry();
        leftFront.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(power);
        rightBack.setPower(power);
    }

    /**
     * Converts the average distance of the range sensors (filtering bad values) into a usable servo power that will put our rack and pinion pusher right in front of the beacon.
     * Uses a linear function that converts the distance from the wall to distance from beacon, then scales it using a slope to determine power.
     * Yay math!!!!
     * @return
     * Returns the power for the servo
     * @throws InterruptedException
     */
    public double getPowerDist() throws InterruptedException{
        double cmBack;
        double cmFront;
        while(true) {
            cmBack = rangeSensorBack.getDistance(DistanceUnit.CM);
            cmFront = rangeSensorFront.getDistance(DistanceUnit.CM);
            if(cmBack==255 && cmFront ==255) {
                idle();
                continue;
            }
            else {
                break;
            }
        }
        double dist;
        if(cmBack!=255 && cmFront!=255) {
            dist =(cmBack + cmFront) / 2.0;
        }
        else if(cmBack!=255) {
            dist = cmBack;
        }
        else {
            dist = cmFront;
        }
        double beaconDist = dist-4;
        double pow = beaconDist*(.36/9);
        return pow;
    }

    /**
     * Once parallel to the wall, the robot goes forward or backwards based on the boolean isReversed, and waits until it sees the color provided in colorStr.
     * It then pushes the button for the right color.
     * @param isReversed
     * Flag for whether the robot is going backwards or forwards
     * @param colorStr
     * Color to look for
     * @throws InterruptedException
     */
    public void doBeacon(boolean isReversed,String colorStr) throws InterruptedException{
        int mult = 1;
        if(isReversed) mult = -1;
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 5 && !getColor(color).equals(colorStr)) {
            double rFront = rangeSensorFront.getDistance(DistanceUnit.CM);
            double rBack = rangeSensorBack.getDistance(DistanceUnit.CM);
            if(rFront - rBack > 1){

                leftFront.setPower(mult*.25);
                leftBack.setPower(mult*.25);
                rightFront.setPower(mult*.15);
                rightBack.setPower(mult*.15);

            }
            else if(rBack - rFront > 1){

                leftFront.setPower(mult*.15);
                leftBack.setPower(mult*.15);
                rightFront.setPower(mult*.25);
                rightBack.setPower(mult*.25);

            }
            else{
                leftFront.setPower(mult*.15);
                leftBack.setPower(mult*.15);
                rightFront.setPower(mult*.15);
                rightBack.setPower(mult*.15);

            }
            autoBeaconSlider.setPower(getPowerDist()-.05);
            idle();
        }
        stopAll();
        if(isReversed) {
            encoderDrive(.15,.15,120,120,2);
            //beaconCorrection(!isReversed,colorStr);
        }
        else {
            encoderDrive(-.15,-.15,80,80,2);
            //beaconCorrection(!isReversed,colorStr);
        }
        autoBeaconSlider.setPower(getPowerDist() + .4);
        sleep(1500);
        autoBeaconSlider.setPower(getPowerDist() - .05);
        sleep(650);
    }

    void beaconTeleop(boolean isReversed,String colorStr) throws InterruptedException{
        int mult = 1;
        if(isReversed) mult = -1;
        if(isReversed) mult = -1;
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 5 && !getColor(color).equals("other")) {
            leftFront.setPower(mult*.17);
            leftBack.setPower(mult*.17);
            rightFront.setPower(mult*.17);
            rightBack.setPower(mult*.17);
            autoBeaconSlider.setPower(getPowerDist()-.1);
            idle();
        }
        stopAll();
        if(isReversed) {
//            encoderDrive(.15,.15,120,120,2);
            beaconCorrection(!isReversed,colorStr);
        }
        else {
//            encoderDrive(-.15,-.15,80,80,2);
            beaconCorrection(!isReversed,colorStr);
        }
        autoBeaconSlider.setPower(getPowerDist()+.4);
        sleep(1500);
        autoBeaconSlider.setPower(getPowerDist()-.05);
        sleep(650);

    }

    void beaconsTeleop() throws InterruptedException{
        balance();
        beaconTeleop(false, "woo");
        balance();
        beaconTeleop(false, "woo");
    }

    public void beaconCorrection(boolean isReversed,String colorStr) throws InterruptedException{
        int mult = 1;
        if(isReversed) mult = -1;
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 5 && !getColor(color).equals(colorStr)) {
            leftFront.setPower(mult*.12);
            leftBack.setPower(mult*.12);
            rightFront.setPower(mult*.12);
            rightBack.setPower(mult*.12);
            autoBeaconSlider.setPower(getPowerDist()-.05);
            idle();
        }
        stopAll();
    }
}
