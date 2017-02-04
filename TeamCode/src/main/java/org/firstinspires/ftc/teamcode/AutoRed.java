package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by wross on 12/16/16.
 */
@Autonomous(name="Red 2 Beacon", group="Beacon Autonomous")
public class AutoRed extends LinearOpMode{
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
    ColorSensor color;

    OurRangeSensor rangeSensorFront;
    OurRangeSensor rangeSensorBack;
    I2cDeviceSynch rangeSensorReaderBack;
    I2cDevice rangeSensorBackDevice;

    I2cDeviceSynch rangeSensorReaderFront;
    I2cDevice rangeSensorFrontDevice;
    MasqAdafruitIMU imu;

    public void initialize(){
        imu = new MasqAdafruitIMU("IMU", hardwareMap);
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
        autoBeaconSlider.setPower(-.7);


        rightBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.REVERSE);

        color = hardwareMap.colorSensor.get("color");
        color.setI2cAddress(I2cAddr.create8bit(0x5a));

        rangeSensorBackDevice = hardwareMap.i2cDevice.get("rangeSensorBack");
        rangeSensorReaderBack = new I2cDeviceSynchImpl(rangeSensorBackDevice, I2cAddr.create8bit(0x38), false);
        rangeSensorFrontDevice = hardwareMap.i2cDevice.get("rangeSensorFront");
        rangeSensorReaderFront = new I2cDeviceSynchImpl(rangeSensorFrontDevice, I2cAddr.create8bit(0x28), false);
        rangeSensorBack = new OurRangeSensor(rangeSensorReaderBack,I2cAddr.create8bit(0x38));
        rangeSensorFront = new OurRangeSensor(rangeSensorReaderFront, I2cAddr.create8bit(0x28));
        telemetry.addData("Initialization ", "complete");
        telemetry.update();
    }
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
    /*public void gyroRight(int degs, double speed) throws InterruptedException {
        double target = (gyro.getHeading()+degs)%360;
        while (Math.abs(gyro.getHeading() - target) > 4 && opModeIsActive()) {
            leftFront.setPower(speed);
            rightFront.setPower(-speed);
            leftBack.setPower(speed);
            rightBack.setPower(-speed);
            telemetry();
            idle();
        }
    }
    public void gyroLeft(int degs, double speed) throws InterruptedException {
        double target = (gyro.getHeading()-degs)%360;
        while (Math.abs(gyro.getHeading() - target) > 4 && opModeIsActive()) {
            leftFront.setPower(-speed);
            rightFront.setPower(speed);
            leftBack.setPower(-speed);
            rightBack.setPower(speed);
            telemetry();
            idle();
        }
    }*/
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

    public void left(double power) throws InterruptedException {
        telemetry();
        leftFront.setPower(-power);
        leftBack.setPower(-power);
        rightFront.setPower(power);
        rightBack.setPower(power);
    }

    public void right(double power) throws InterruptedException {
        telemetry();
        leftFront.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(-power);
        rightBack.setPower(-power);
    }

    public void balance() throws InterruptedException {
        runtime.reset();
        while (Math.abs(rangeSensorFront.getDistance(DistanceUnit.CM) - rangeSensorBack.getDistance(DistanceUnit.CM)) > 1 && opModeIsActive() && runtime.seconds() < 3){
            double cmBack = rangeSensorBack.getDistance(DistanceUnit.CM);
            double cmFront = rangeSensorFront.getDistance(DistanceUnit.CM);
            if(cmBack==255 || cmFront ==255) {
                stopAll();
                continue;
            }
            if (cmFront > cmBack) {
                right(.21);
            }
            if (rangeSensorFront.getDistance(DistanceUnit.CM) < rangeSensorBack.getDistance(DistanceUnit.CM)) {
                left(.21);
            }
            telemetry();
            autoBeaconSlider.setPower(getPowerDist());
            idle();
        }
        stopAll();
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
                if(Math.abs(leftFront.getCurrentPosition()) > Math.abs(leftCounts)*(7.0/8) || Math.abs(leftBack.getCurrentPosition()) > Math.abs(leftCounts)*(7.0/8) || Math.abs(rightFront.getCurrentPosition()) > Math.abs(rightCounts)*(7.0/8) || Math.abs(rightBack.getCurrentPosition()) > Math.abs(rightCounts)*(7.0/8)) {
                    leftFront.setPower(.25*leftSpeed);
                    rightFront.setPower(.25*rightSpeed);
                    leftBack.setPower(.25*leftSpeed);
                    rightBack.setPower(.25*rightSpeed);
                    telemetry.addData("speed",.5);
                }
                if(Math.abs(leftFront.getCurrentPosition()) > Math.abs(leftCounts)*(.5) || Math.abs(leftBack.getCurrentPosition()) > Math.abs(leftCounts)*(.5) || Math.abs(rightFront.getCurrentPosition()) > Math.abs(rightCounts)*(.5) || Math.abs(rightBack.getCurrentPosition()) > Math.abs(rightCounts)*(.5)) {
                    leftFront.setPower(.4*leftSpeed);
                    rightFront.setPower(.4*rightSpeed);
                    leftBack.setPower(.4*leftSpeed);
                    rightBack.setPower(.4*rightSpeed);
                    telemetry.addData("speed",.5);
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
    public void telemetry(){
        telemetry.addData(imu.getName(), imu.telemetrize());
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
    public void shootBall() throws InterruptedException{
        shooter.setPower(1);
        sleep(1000);
        shooter.setPower(0);
    }
    public boolean isIn(){
        return ballCheck.getLightDetected() > .014;
    }
    public boolean intake() throws InterruptedException{
        pickpMechanism.setPower(1);
        runtime.reset();
        while(!isIn() && runtime.seconds() < 5 && opModeIsActive()) idle();
        pickpMechanism.setPower(0);
        return true;
    }
    public void forward(double power) throws InterruptedException{
        telemetry();
        leftFront.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(power);
        rightBack.setPower(power);
    }
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
    public void doBeacon(boolean isReversed) throws InterruptedException{
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 5 && !getColor(color).equals("red")) {
            if(isReversed){
                forward(-.1);
            }
            else{
                forward(.1);
            }
            autoBeaconSlider.setPower(getPowerDist()-.05);
            idle();
        }
        stopAll();
        autoBeaconSlider.setPower(getPowerDist()+.4);
        sleep(1000);
        autoBeaconSlider.setPower(getPowerDist()-.05);
        sleep(650);
    }

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
        encoderDrive(-.75,.75,800,800,3);
//        imuLeft(70,.35);
        idle();
        sleep(100);
        encoderDrive(.14,.14,234234,234234,1.5);
        encoderDrive(-.19,-.19,125,125,3);
        autoBeaconSlider.setPower(0);

//        imuLeft(10,.35);
        encoderDrive(-.75,.75,400,400,3);
        autoBeaconSlider.setPower(getPowerDist());
        sleep(500);
        idle();
        balance();
        autoBeaconSlider.setPower(getPowerDist());
//        sleep(500);
        doBeacon(true);
        idle();
        balance();
        autoBeaconSlider.setPower(getPowerDist());
        encoderDrive(.4,.4,1700,1700,5);
//        balance();
        doBeacon(false);
        encoderDrive(-.5,.5,100,100,2);
        encoderDrive(.5,.5,1600,1600,5);

    }


}