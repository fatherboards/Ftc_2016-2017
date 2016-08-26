package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

public class GoToLineTurnArround extends  OpMode {
    int heading;
    int headingCur;
    boolean mustTurn = false;
    ColorSensor colorSensor;
    GyroSensor sensorGyro;
    DcMotor motorRight;
    DcMotor motorLeft;
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
    }
    public void loop(){
        colorSensor.enableLed(true);
        float red = colorSensor.red();
        float blue = colorSensor.blue();
        float green = colorSensor.green();
        telemetry.addData("Red  ", red);
        if(red > 7){
            heading = sensorGyro.getHeading();
            mustTurn = true;
        }
        if(mustTurn == true) {
            motorLeft.setPower(.05);
            motorRight.setPower(.05);
            headingCur = sensorGyro.getHeading();
            telemetry.addData("H", headingCur);
            if(headingCur > 180 && headingCur< 190){
                mustTurn = false;
            }
        }
        else{
            motorRight.setPower(-.2);
            motorLeft.setPower(.2);
        }

    }

}
