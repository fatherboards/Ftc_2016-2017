package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.Robot_API_Abstraction;

/**
 * Created by wyatt.ross on 9/12/16.
 */
public class LineFollow extends Robot_API_Abstraction{
    ColorSensor colorLeft;
    ColorSensor colorRight;
    public void init(){
        colorLeft = hardwareMap.colorSensor.get("leftC");
        colorRight = hardwareMap.colorSensor.get("rightC");
    }

    public void loop(){
        if(ColorSensorOneInt(colorLeft, true, "left") > 14.25){
            telemetry.addData("DirectionTurning: ", "right");
        }
        if(ColorSensorOneInt(colorRight, true, "right") > 14.25){
            telemetry.addData("DirectionTurning: ", "left");
        }
    }
}
