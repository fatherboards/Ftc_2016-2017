package org.firstinspires.ftc.teamcode;

import android.text.method.Touch;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.Robot_API_Abstraction;
import java.util.ArrayList;

/**
 * Created by wyatt.ross on 9/12/16.
 */

public class GoUntilPressed extends Robot_API_Abstraction{
    //Motors and Sensor
    DcMotor motorLeft;
    DcMotor motorRight;
    TouchSensor touch;
    ArrayList<DcMotor> motors = new ArrayList<DcMotor>();

    //Start Constructor
    public GoUntilPressed(){

    }
    //End Constructor

    public void init(){
        motorLeft = hardwareMap.dcMotor.get("left");
        motorRight = hardwareMap.dcMotor.get("right");
        touch = hardwareMap.touchSensor.get("touch");
        motors.add(motorLeft);
        motors.add(motorRight);
    }

    public void loop(){
        ArrayList<Float> motorPower = new ArrayList<Float>();
        //Keeps going until touch sensor is pressed
        if(TouchSensorData(touch, "isPressed") == true){
            stop();
        }
        else{
            motorPower.add((float).25);
            motorPower.add((float)-.25);
        }
        setMotorsPower(motors, motorPower);
    }

    public void stop(){

    }

}
