package com.qualcomm.ftcrobotcontroller.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;
public class TS extends OpMode{
    public TS(){

    }
    //define touch sensor
    TouchSensor touchSensor;
    @Override
    public void init() {
        //Maps to config file
        touchSensor = hardwareMap.touchSensor.get("t");
    }
    @Override
    public void loop(){
        //Sends button state to Driver station
        telemetry.addData("isPressed", String.valueOf(touchSensor.isPressed()));
    }
}