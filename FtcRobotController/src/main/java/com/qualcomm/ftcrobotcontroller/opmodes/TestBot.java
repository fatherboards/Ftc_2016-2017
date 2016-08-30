package com.qualcomm.ftcrobotcontroller.opmodes;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceReader;

import java.util.ArrayList;

/**
 * Created by wyatt.ross on 8/24/16.
 */

public class TestBot extends OpMode{

    //declares motors
    DcMotor motorRight;
    DcMotor motorLeft;

    //CDI and attached objects declared
    ColorSensor colorSensor;
    I2cDevice range;
    I2cDeviceReader rangeReader = new I2cDeviceReader(range, 0x28, 0x04, 2);
    DeviceInterfaceModule cdim;
    TouchSensor t;
    GyroSensor sensorGyro;
    IrSeekerSensor irSeeker;

    //Global Variables
    ArrayList<Thread> runningThreads = new ArrayList<Thread>();
    byte rangeReadings[];
    float hsvValues[] = {0,0,0};
    int xVal, yVal, zVal = 0;
    int heading = 0;

    //Lists devices and declares the device
    public enum ColorSensorDevice {ADAFRUIT, HITECHNIC_NXT, MODERN_ROBOTICS_I2C};
    public ColorSensorDevice device = ColorSensorDevice.MODERN_ROBOTICS_I2C;


    // Constructor
    public TestBot() {

    }
    // End Constructor

    public void init(){
        //Hardware Mapping
        hardwareMap.logDevices();
        range = hardwareMap.i2cDevice.get("range");
        cdim = hardwareMap.deviceInterfaceModule.get("dim");
        colorSensor = hardwareMap.colorSensor.get("color");
        t = hardwareMap.touchSensor.get("t");
        sensorGyro = hardwareMap.gyroSensor.get("gyro");
        motorRight = hardwareMap.dcMotor.get("motor_2");
        motorLeft = hardwareMap.dcMotor.get("motor_1");
        irSeeker = hardwareMap.irSeekerSensor.get("ir");
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        colorSensor.enableLed(false);
        //Allows robot to move
        Thread r = new Thread() {
            public void run() {
                while (true) {
                    float left = -gamepad1.left_stick_y;
                    float right = -gamepad1.right_stick_y;
                    right = Range.clip(right, -1, 1);
                    left = Range.clip(left, -1, 1);
                    right = (float) scaleInput(right);
                    left = (float) scaleInput(left);
                    motorRight.setPower(right);
                    motorLeft.setPower(left);

                }
            }
        };

        //Sensor Activity
        Thread h = new Thread(){
            public void run() {
                while(true) {
                    //ir seeker
                    double angle = irSeeker.getAngle();
                    double strength = irSeeker.getStrength();
                    telemetry.addData("angle", angle);
                    telemetry.addData("strength", strength);

                    //range sensor
                    rangeReadings = rangeReader.getReadBuffer();
                    telemetry.addData("US", (rangeReadings[0] & 0xFF));
                    telemetry.addData("ODS", (rangeReadings[1] & 0xFF));

                    //touch sensor
                    telemetry.addData("isPressed", String.valueOf(t.isPressed()));

                    //color sensor
                    colorSensor.enableLed(true);
                    float red = colorSensor.red();
                    float blue = colorSensor.blue();
                    float green = colorSensor.green();
                    Color.RGBToHSV(colorSensor.red()*8, colorSensor.green()*8, colorSensor.blue()*8, hsvValues);
                    telemetry.addData("Red  ", red);
                    telemetry.addData("Green", green);
                    telemetry.addData("Blue ", blue);
                    telemetry.addData("Hue", hsvValues[0]);
                    telemetry.addData("Clear", colorSensor.alpha());
                }
            }
        };
        runningThreads.add(h);
        h.start();
        runningThreads.add(r);
        r.start();
    }

    public void loop(){}

    public void stop(){
        //Allows us to stop the robot with the stop button, canceling the run without this will not stop the other threads
        for(Thread t : runningThreads) {
            t.interrupt();
        }
    }

    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };
            int index = (int) (dVal * 16.0);
        if (index < 0) {
            index = -index;
        }
        if (index > 16) {
            index = 16;
        }
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }
        return dScale;
    }
}
