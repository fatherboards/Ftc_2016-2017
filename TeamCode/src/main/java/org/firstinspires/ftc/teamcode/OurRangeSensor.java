package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cAddrConfig;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;

/**
 * {@link com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor} implements support for the MR ultrasonic/optical combo
 * range sensor.
 *
 * @see <a href="http://www.modernroboticsinc.com/range-sensor">MR Range Sensor</a>
 */
@I2cSensor(name = "Our Range Sensor", description = "a MR range sensor", xmlTag = "OurRangeSensor")
public class OurRangeSensor extends ModernRoboticsI2cRangeSensor implements DistanceSensor, OpticalDistanceSensor, I2cAddrConfig
{
    public OurRangeSensor(I2cDeviceSynch deviceClient, I2cAddr addr)
    {
        super(deviceClient);

        this.setI2cAddress(addr);
        this.deviceClient.engage();
    }
}