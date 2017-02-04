package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by zipper on 2/3/17.
 */
@TeleOp
public class VariableExtensionTest extends LinearOpMode {
    private CRServo autoBeaconSlider;
    OurRangeSensor rangeSensorFront;
    OurRangeSensor rangeSensorBack;
    I2cDeviceSynch rangeSensorReaderBack;
    I2cDevice rangeSensorBackDevice;
    I2cDeviceSynch rangeSensorReaderFront;
    I2cDevice rangeSensorFrontDevice;

    public double getPowerDist() throws InterruptedException{
        double cmBack;
        double cmFront;
        while(true) {
            cmBack = rangeSensorBack.getDistance(DistanceUnit.CM);
            cmFront = rangeSensorFront.getDistance(DistanceUnit.CM);
            if(cmBack==255 && cmFront ==255) {
                idle();
                continue;
            }
            else {
                break;
            }
        }
        double dist;
        if(cmBack!=255 && cmFront!=255) {
            dist =(cmBack + cmFront) / 2.0;
        }
        else if(cmBack!=255) {
            dist = cmBack;
        }
        else {
            dist = cmFront;
        }
        double beaconDist = dist-4;
        double pow = beaconDist*(.36/9);
        return pow;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        rangeSensorBackDevice = hardwareMap.i2cDevice.get("rangeSensorBack");
        rangeSensorReaderBack = new I2cDeviceSynchImpl(rangeSensorBackDevice, I2cAddr.create8bit(0x38), false);
        rangeSensorFrontDevice = hardwareMap.i2cDevice.get("rangeSensorFront");
        rangeSensorReaderFront = new I2cDeviceSynchImpl(rangeSensorFrontDevice, I2cAddr.create8bit(0x28), false);
        rangeSensorBack = new OurRangeSensor(rangeSensorReaderBack,I2cAddr.create8bit(0x38));
        rangeSensorFront = new OurRangeSensor(rangeSensorReaderFront, I2cAddr.create8bit(0x28));
        autoBeaconSlider = hardwareMap.crservo.get("slider");
        telemetry.addData("Initialization ", "complete");
        telemetry.update();
        while (!(isStarted() || isStopRequested())) {
            idle();
        }
        waitForStart();
        while(opModeIsActive()){
            autoBeaconSlider.setPower(getPowerDist());
            telemetry.addData("Range Sensor Front", "%.2f cm", rangeSensorFront.getDistance(DistanceUnit.CM));
            telemetry.addData("Range Sensor Back", "%.2f cm", rangeSensorBack.getDistance(DistanceUnit.CM));
            telemetry.update();
            idle();
        }
    }

}
