package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.Robot_API_Abstraction;
import java.util.ArrayList;

/**
 * Created by wyatt.ross on 9/10/16.
 */


@TeleOp(name = "Teleop", group="Teleop")
public class TeleopVelocityVortex extends Robot_API_Abstraction{
    boolean foundLine = false;
    //Motors
    DcMotor motorRight;
    DcMotor motorLeft;
    DcMotor motorRight2;
    DcMotor motorLeft2;
    ArrayList<DcMotor> motors = new ArrayList<DcMotor>();


    //Start constructor
    public TeleopVelocityVortex(){

    }
    //End constructor

    public void init(){
        //Hardware mapping and setup
        hardwareMap.logDevices();
        motorRight = hardwareMap.dcMotor.get("motorRight");
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight2 = hardwareMap.dcMotor.get("motorRight2");
        motorLeft2 = hardwareMap.dcMotor.get("motorLeft2");
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        motors.add(motorRight);
        motors.add(motorLeft);
        motors.add(motorRight2);
        motors.add(motorLeft2);
    }

    public void loop(){
        ArrayList<Float> motorPowers = new ArrayList<Float>();
        double right = gamepad1.right_stick_y;
        double left = gamepad1.left_stick_y;
        motorPowers.add((float)right);
        motorPowers.add((float)left);
        motorPowers.add((float)-right);
        motorPowers.add((float)left);
        setMotorsPower(motors, motorPowers);
    }
}
