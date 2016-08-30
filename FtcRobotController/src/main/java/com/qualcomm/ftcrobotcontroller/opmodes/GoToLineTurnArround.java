package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;

public class GoToLineTurnArround extends  OpMode {
    //Global Variable
    int heading; //used for loop code
    int headingCur; //used for loop code
    boolean mustTurn = false; //used for loop code
    ArrayList<Thread> runningThreads = new ArrayList<Thread>();

    //Hardware
    ColorSensor colorSensor;
    GyroSensor sensorGyro;
    DcMotor motorRight;
    DcMotor motorLeft;

    //Constructor
    public GoToLineTurnArround(){}
    //End Constructor

    public void init(){
        //hardwareMapping
        motorRight = hardwareMap.dcMotor.get("motor_2");
        motorLeft = hardwareMap.dcMotor.get("motor_1");
        hardwareMap.logDevices();
        colorSensor = hardwareMap.colorSensor.get("color");
        colorSensor.enableLed(false);
        sensorGyro = hardwareMap.gyroSensor.get("gyro");

        //initialization (gyro calibration) and encoder setup
        sensorGyro.calibrate();
        while (sensorGyro.isCalibrating())  {
            continue;
        }
        setDriveMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        //run program
        goForwardDistance(10000);
    }
    public void loop(){
        //This will be uncommented LATER (maybe)
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
        //put on new thread in order to run from initialization
        Thread t = new Thread() {
            public void run() {

                //Relevant variables
                int currentEncodersRight = motorRight.getCurrentPosition();
                int currentEncodersLeft = motorLeft.getCurrentPosition();
                int endEncodersRight = -encoderTicks-currentEncodersRight;
                int endEncodersLeft = encoderTicks+currentEncodersLeft;
                int startGyro = sensorGyro.getHeading();
                int gyro = startGyro;

                //sets distance to go from parameter
                motorLeft.setTargetPosition(endEncodersLeft);
                motorRight.setTargetPosition(endEncodersRight);

                while(motorLeft.getCurrentPosition() < endEncodersLeft) {

                    //all telemetry
                    telemetry.addData("encodersLeft",-motorLeft.getCurrentPosition()-endEncodersLeft);
                    telemetry.addData("encodersRight",motorRight.getCurrentPosition()-endEncodersRight);
                    telemetry.addData("4. h", String.format("%03d", gyro - startGyro));

                    gyro = sensorGyro.getHeading(); //this will only be called if the robot has already had its course corrected(with the exception of the first loopthrough) it therefore does not need a conditional

                    //Course Correction
                    while(sensorGyro.getHeading() < gyro - 10) {
                        motorRight.setPower(-.1);
                        motorLeft.setPower(-.1);
                    }
                    while(sensorGyro.getHeading() > gyro + 10) {
                        motorRight.setPower(.1);
                        motorLeft.setPower(.1);
                    }
                    //end course correction

                    //goes straight until the condition of a while loop is met
                    motorRight.setPower(-.1);
                    motorLeft.setPower(.1);
                }
                //Irellevant but for redundancy
                motorLeft.setPower(0);
                motorRight.setPower(0);

            }

        };

        //starts thread
        runningThreads.add(t);
        t.start();
    }

    public void stop() {
        //Allows us to stop the robot with the stop button, canceling the run without this will not stop the other threads
        for(Thread t : runningThreads) {
            t.interrupt();
        }
    }

    //Made for simplicity of init
    public void setDriveMode(DcMotorController.RunMode mode) {
        if (motorLeft.getChannelMode() != mode) {
            motorLeft.setChannelMode(mode);
        }

        if (motorRight.getChannelMode() != mode) {
            motorRight.setChannelMode(mode);
        }
    }
}
