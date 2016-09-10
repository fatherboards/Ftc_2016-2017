package com.qualcomm.ftcrobotcontroller;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceReader;
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

    public void ColorSensorData(ColorSensor colorSensor){
        colorSensor.enableLed(true);
        float red = colorSensor.red();
        float blue = colorSensor.blue();
        float green = colorSensor.green();
        telemetry.addData("Red  ", red);
        telemetry.addData("Green", green);
        telemetry.addData("Blue ", blue);
        telemetry.addData("Clear", colorSensor.alpha());
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
            setDriveMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS, motors.get(i));
            motors.get(i).setTargetPosition(TargetPositions.get(i));
            motors.get(i).setPower(motorPowers.get(i));
        }
    }

    public void setDriveMode(DcMotorController.RunMode mode,DcMotor motor) {
        if (motor.getChannelMode() != mode) {
            motor.setChannelMode(mode);
        }
    }

    public void setServoPositions(ArrayList<Servo> Servos, ArrayList<Double> servoPositions){
        for(int i = 0; i < Servos.size(); i++){
            Servos.get(i).setPosition(servoPositions.get(i));
        }
    }

    public void initI2CDeviceReader(I2cDevice device, I2cDeviceReader readings,int hex1, int hex2, int integer){
        readings = new I2cDeviceReader(device, hex1, hex2, integer);
    }


}
