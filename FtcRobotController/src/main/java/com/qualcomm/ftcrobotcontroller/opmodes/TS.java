package com.qualcomm.ftcrobotcontroller.opmodes;

import android.app.Activity;
import android.graphics.Color;
import android.text.method.Touch;
import android.view.View;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.TouchSensor;
public class TS extends OpMode{
    public TS(){

    }
    TouchSensor touchSensor;
    @Override
    public void init() {
        touchSensor = hardwareMap.touchSensor.get("t");
    }
    @Override
    public void loop(){
        telemetry.addData("isPressed", String.valueOf(touchSensor.isPressed()));
    }
}