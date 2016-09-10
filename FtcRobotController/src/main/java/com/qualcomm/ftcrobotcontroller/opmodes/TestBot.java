package com.qualcomm.ftcrobotcontroller.opmodes;
import com.qualcomm.ftcrobotcontroller.Robot_API_Abstraction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
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

public class TestBot extends Robot_API_Abstraction{

    //declares motors
    DcMotor motorRight;
    DcMotor motorLeft;
    ArrayList<DcMotor> DcMotorList = new ArrayList<DcMotor>();

    //CDI and attached objects declared
    ColorSensor colorSensor;
    I2cDevice range;
    I2cDeviceReader rangeReader;
    DeviceInterfaceModule cdim;
    TouchSensor t;
    GyroSensor sensorGyro;
    IrSeekerSensor irSeeker;

    //Global Variables
    ArrayList<Thread> runningThreads = new ArrayList<Thread>();
    byte rangeReadings[];

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

        //initialization
        rangeReader = new I2cDeviceReader(range, 0x28, 0x04, 2);

        DcMotorList.add(motorRight);
        DcMotorList.add(motorLeft);

        //Motor Thread
        Thread r = new Thread() {
            public void run() {
                while (true) {
                    Float right = -gamepad1.right_stick_y;
                    Float left = gamepad1.left_stick_y;
                    ArrayList<Float> motorPowerList = new ArrayList<Float>();
                    motorPowerList.add(right);
                    motorPowerList.add(left);
                    setMotorsPower(DcMotorList, motorPowerList);

                }
            }
        };

        //Sensor Activity
        Thread h = new Thread(){
            public void run() {
                while(true) {
                    //ir seeker
                    IrSeeker(irSeeker, "Angle", "Power");

                    //touch sensor
                    TouchSensorData(t, "TouchSensor");

                    //gyro sensor
                    GyroData(sensorGyro, "heading");

                    //color sensor
                    ColorSensorData(colorSensor);
                }
            }
        };

        //Start Activities
        runningThreads.add(h);
        h.start();
        runningThreads.add(r);
        r.start();
    }

    public void loop(){
        //range sensor (Cannot put into sensor thread, it causes a crash for some reason)
        RangeSensorActivity(rangeReader, rangeReadings, "ultraSonicDistance", "opticalDistance");
    }

    public void stop(){
        //Allows us to stop the robot with the stop button, canceling the run without this will not stop the other threads
        for(Thread t : runningThreads) {
            t.interrupt();
        }
    }
}
