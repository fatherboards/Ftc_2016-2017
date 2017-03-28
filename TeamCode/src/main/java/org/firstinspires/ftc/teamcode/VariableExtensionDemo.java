package org.firstinspires.ftc.teamcode;

/**
 * Created by zipper on 2/3/17.
 *
 * Demo for judges that shows off our variable extension
 */
//@TeleOp
public class VariableExtensionDemo extends FatherboardsLinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        while (!(isStarted() || isStopRequested())) {
            idle();
        }
        waitForStart();
        while(opModeIsActive()){
            autoBeaconSlider.setPower(getPowerDist());
            telemetry();
            idle();
        }
    }

}
