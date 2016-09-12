package org.firstinspires.ftc;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.I2cDeviceReader;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;

import java.util.ArrayList;



public abstract class Robot_API_Abstraction extends OpMode{
    public void setMotorsPower(ArrayList<DcMotor> motors, ArrayList<Float> motorPowers){ //set power to many motors
        for(int i = 0; i < motors.size(); i++){
            motors.get(i).setPower(motorPowers.get(i));
        }

    }
    public void RangeSensorActivity(I2cDeviceReader rangeReader, byte readings[], String UltraSonicTelemetryName, String OpticalDistanceTelemetryName){
        readings = rangeReader.getReadBuffer();
        telemetry.addData(UltraSonicTelemetryName, (readings[0] & 0xFF));
        telemetry.addData(OpticalDistanceTelemetryName, (readings[1] & 0xFF));
    }

    public void ColorSensorData(ColorSensor colorSensor, boolean Mode){
        colorSensor.enableLed(Mode);
        float red = colorSensor.red();
        float blue = colorSensor.blue();
        float green = colorSensor.green();
        telemetry.addData("Red  ", red);
        telemetry.addData("Green", green);
        telemetry.addData("Blue ", blue);
        telemetry.addData("Clear", colorSensor.alpha());
    }

    public float ColorSensorOneInt(ColorSensor colorSensor, boolean Mode, String TelemetryString){
        colorSensor.enableLed(Mode);
        float hue = colorSensor.argb();
        telemetry.addData(TelemetryString, hue);
        return hue;
    }

    public void TouchSensorData(TouchSensor t, String TelemetryName){
        telemetry.addData(TelemetryName, String.valueOf(t.isPressed()));
    }

    public void GyroData(GyroSensor Gyro, String HeadingTelemetryName){
        int heading = Gyro.getHeading();
        telemetry.addData(HeadingTelemetryName, heading);
    }

    public void IrSeeker(IrSeekerSensor ir, String AngleNameTelemetry, String PowerTelemetryName){
        double angle = ir.getAngle();
        double strength = ir.getStrength();
        telemetry.addData("angle", angle);
        telemetry.addData("strength", strength);
    }

    public void GoEncoderTicks(ArrayList<DcMotor> motors, ArrayList<Integer> TargetPositions, ArrayList<Double> motorPowers){
        for(int i = 0; i < motors.size(); i++) {
            motors.get(i).setTargetPosition(TargetPositions.get(i));
            motors.get(i).setPower(motorPowers.get(i));
        }
    }

    public void setDriveMode(int mode,DcMotor motor) {
        DcMotor.RunMode[] modes = new DcMotor.RunMode[]
                {
                        DcMotor.RunMode.RUN_WITHOUT_ENCODER,        // 1
                        DcMotor.RunMode.RUN_USING_ENCODER,          // 2
                        DcMotor.RunMode.STOP_AND_RESET_ENCODER,     // 3

                        DcMotor.RunMode.RUN_WITHOUT_ENCODER,        // 4
                        DcMotor.RunMode.STOP_AND_RESET_ENCODER,     // 5
                        DcMotor.RunMode.RUN_USING_ENCODER,          // 6

                        DcMotor.RunMode.RUN_USING_ENCODER,          // 7
                        DcMotor.RunMode.RUN_WITHOUT_ENCODER,        // 8
                        DcMotor.RunMode.STOP_AND_RESET_ENCODER,     // 9

                        DcMotor.RunMode.RUN_USING_ENCODER,          // 10
                        DcMotor.RunMode.STOP_AND_RESET_ENCODER,     // 11
                        DcMotor.RunMode.RUN_WITHOUT_ENCODER,        // 12

                        DcMotor.RunMode.STOP_AND_RESET_ENCODER,     // 13
                        DcMotor.RunMode.RUN_WITHOUT_ENCODER,        // 14
                        DcMotor.RunMode.RUN_USING_ENCODER,          // 15

                        DcMotor.RunMode.STOP_AND_RESET_ENCODER,     // 3
                        DcMotor.RunMode.RUN_USING_ENCODER,          // 2
                        DcMotor.RunMode.RUN_WITHOUT_ENCODER,        // 1
                };
            DcMotor.RunMode finalMode = modes[mode];
            motor.setMode(finalMode);
    }

    public void setServoPositions(ArrayList<Servo> Servos, ArrayList<Double> servoPositions){
        for(int i = 0; i < Servos.size(); i++){
            Servos.get(i).setPosition(servoPositions.get(i));
        }
    }
}
