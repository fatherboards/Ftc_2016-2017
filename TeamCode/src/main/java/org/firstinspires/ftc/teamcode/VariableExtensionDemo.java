package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by zipper on 2/3/17.
 *
 * Demo for judges that shows off our variable extension
 */
@TeleOp
public class VariableExtensionDemo extends FatherboardsLinearOpMode {
    private CRServo autoBeaconSlider;
    private FatherboardsRangeSensor rangeSensorFront;
    private FatherboardsRangeSensor rangeSensorBack;
    private I2cDeviceSynch rangeSensorReaderBack;
    private I2cDevice rangeSensorBackDevice;
    private I2cDeviceSynch rangeSensorReaderFront;
    private I2cDevice rangeSensorFrontDevice;

    @Override
    public void runOpMode() throws InterruptedException {
        rangeSensorBackDevice = hardwareMap.i2cDevice.get("rangeSensorBack");
        rangeSensorReaderBack = new I2cDeviceSynchImpl(rangeSensorBackDevice, I2cAddr.create8bit(0x38), false);
        rangeSensorFrontDevice = hardwareMap.i2cDevice.get("rangeSensorFront");
        rangeSensorReaderFront = new I2cDeviceSynchImpl(rangeSensorFrontDevice, I2cAddr.create8bit(0x28), false);
        rangeSensorBack = new FatherboardsRangeSensor(rangeSensorReaderBack,I2cAddr.create8bit(0x38));
        rangeSensorFront = new FatherboardsRangeSensor(rangeSensorReaderFront, I2cAddr.create8bit(0x28));
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
