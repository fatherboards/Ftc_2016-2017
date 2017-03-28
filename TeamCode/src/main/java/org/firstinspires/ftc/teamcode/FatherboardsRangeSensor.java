package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cAddrConfig;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by Jacob Allen Zipper. Master Programmer and bug hunter.
 *
 * {@link com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor} implements support for the MR ultrasonic/optical combo
 * range sensor.
 *
 * @see <a href="http://www.modernroboticsinc.com/range-sensor">MR Range Sensor</a>
 */
//@I2cSensor(name = "Our Range Sensor", description = "a MR range sensor", xmlTag = "FatherboardsRangeSensor")
public class FatherboardsRangeSensor extends ModernRoboticsI2cRangeSensor implements DistanceSensor, OpticalDistanceSensor, I2cAddrConfig
{
    /**
     * Initialize the range sensor not via hardware mapping it, but by making an I2cDevice, and passing the device and address.
     *
     * Example:
     *      FatherboardsRangeSensor rangeSensor = new FatherboardsRangeSensor(rangeSensorI2cDevice, I2cAddr.create8bit(0x13));
     *
     * @param deviceClient
     * The range sensor I2cDevice
     * @param addr
     * The range sensor I2c Address
     */
    public FatherboardsRangeSensor(I2cDeviceSynch deviceClient, I2cAddr addr)
    {
        super(deviceClient);

        this.setI2cAddress(addr);
        this.deviceClient.engage();
    }
}