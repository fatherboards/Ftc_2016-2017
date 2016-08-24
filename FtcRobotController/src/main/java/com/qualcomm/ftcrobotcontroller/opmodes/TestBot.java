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

/**
 * Created by wyatt.ross on 8/24/16.
 */

public class TestBot extends OpMode{

    //declares motors
    DcMotor motorRight;
    DcMotor motorLeft;

    //CDI and attached objects declared
    ColorSensor colorSensor;
    DeviceInterfaceModule cdim;
    TouchSensor t;
    GyroSensor sensorGyro;
    IrSeekerSensor irSeeker;

    //Global Variables
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
        cdim = hardwareMap.deviceInterfaceModule.get("dim");
        colorSensor = hardwareMap.colorSensor.get("color");
        t = hardwareMap.touchSensor.get("t");
        sensorGyro = hardwareMap.gyroSensor.get("gyro");
        motorRight = hardwareMap.dcMotor.get("motor_2");
        motorLeft = hardwareMap.dcMotor.get("motor_1");
        irSeeker = hardwareMap.irSeekerSensor.get("ir");
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        colorSensor.enableLed(false);
    }

    public void loop(){
        //Allows robot to move
        float left = -gamepad1.left_stick_y;
        float right = -gamepad1.right_stick_y;
        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);
        right = (float)scaleInput(right);
        left =  (float)scaleInput(left);
        motorRight.setPower(right);
        motorLeft.setPower(left);

        //Color Sensor Activity
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

        //Gyro Sensor Telemetry
        xVal = sensorGyro.rawX();
        yVal = sensorGyro.rawY();
        zVal = sensorGyro.rawZ();
        heading = sensorGyro.getHeading();
        telemetry.addData("1. x", String.format("%03d", xVal));
        telemetry.addData("2. y", String.format("%03d", yVal));
        telemetry.addData("3. z", String.format("%03d", zVal));
        telemetry.addData("4. h", String.format("%03d", heading));

        //Touch Sensor Telemetry
        telemetry.addData("isPressed", String.valueOf(t.isPressed()));

        //Ir Seeker Activity
        double angle = irSeeker.getAngle();
        double strength = irSeeker.getStrength();
        telemetry.addData("angle", angle);
        telemetry.addData("strength", strength);
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
