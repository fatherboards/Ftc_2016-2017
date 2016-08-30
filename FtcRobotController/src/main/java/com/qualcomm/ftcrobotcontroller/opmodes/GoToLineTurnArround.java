package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;

public class GoToLineTurnArround extends  OpMode {
    int heading;
    int headingCur;
    boolean mustTurn = false;
    ColorSensor colorSensor;
    GyroSensor sensorGyro;
    DcMotor motorRight;
    DcMotor motorLeft;
    ArrayList<Thread> runningThreads = new ArrayList<Thread>();
    public GoToLineTurnArround(){}
    public void init(){
        motorRight = hardwareMap.dcMotor.get("motor_2");
        motorLeft = hardwareMap.dcMotor.get("motor_1");
        hardwareMap.logDevices();
        colorSensor = hardwareMap.colorSensor.get("color");
        colorSensor.enableLed(false);
        sensorGyro = hardwareMap.gyroSensor.get("gyro");
        sensorGyro.calibrate();
        while (sensorGyro.isCalibrating())  {
            continue;
        }
        setDriveMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        goForwardDistance(10000);
    }
    public void loop(){
//        colorSensor.enableLed(true);
//        float red = colorSensor.red();
//        float blue = colorSensor.blue();
//        float green = colorSensor.green();
//        telemetry.addData("Red  ", red);
//        if(red > 7){
//            heading = sensorGyro.getHeading();
//            mustTurn = true;
//        }
//        if(mustTurn == true) {
//            motorLeft.setPower(.05);
//            motorRight.setPower(.05);
//            headingCur = sensorGyro.getHeading();
//            telemetry.addData("H", headingCur);
//            if(headingCur > 180 && headingCur< 190){
//                mustTurn = false;
//            }
//        }
//        else{
//            motorRight.setPower(-.2);
//            motorLeft.setPower(.2);
//        }
    }

    public void goForwardDistance(final int encoderTicks) {
        Thread t = new Thread() {
            public void run() {
                int currentEncodersRight = motorRight.getCurrentPosition();
                int currentEncodersLeft = motorLeft.getCurrentPosition();
                int endEncodersRight = -encoderTicks-currentEncodersRight;
                int endEncodersLeft = encoderTicks+currentEncodersLeft;
                motorLeft.setTargetPosition(endEncodersLeft);
                motorRight.setTargetPosition(endEncodersRight);
                int startGyro = sensorGyro.getHeading();
                int gyro = sensorGyro.getHeading();
                while(motorLeft.getCurrentPosition() < endEncodersLeft) {
                    telemetry.addData("encodersLeft",-motorLeft.getCurrentPosition()-endEncodersLeft);
                    telemetry.addData("encodersRight",motorRight.getCurrentPosition()-endEncodersRight);
                    gyro = sensorGyro.getHeading();
                    while(sensorGyro.getHeading() < gyro - 10) {
                        motorRight.setPower(-.1);
                        motorLeft.setPower(-.1);
                    }
                    while(sensorGyro.getHeading() > gyro + 10) {
                        motorRight.setPower(.1);
                        motorLeft.setPower(.1);
                    }
                    motorRight.setPower(-.1);
                    motorLeft.setPower(.1);
                    heading = sensorGyro.getHeading();
                    telemetry.addData("4. h", String.format("%03d", gyro - startGyro));
                }
                motorLeft.setPower(0);
                motorRight.setPower(0);

            }

        };
        runningThreads.add(t);
        t.start();
    }
    public void stop() {
        for(Thread t : runningThreads) {
            t.interrupt();
        }
    }
    public void setDriveMode(DcMotorController.RunMode mode) {
        if (motorLeft.getChannelMode() != mode) {
            motorLeft.setChannelMode(mode);
        }

        if (motorRight.getChannelMode() != mode) {
            motorRight.setChannelMode(mode);
        }
    }
}
