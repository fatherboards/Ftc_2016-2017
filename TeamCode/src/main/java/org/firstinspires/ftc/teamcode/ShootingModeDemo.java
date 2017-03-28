package org.firstinspires.ftc.teamcode;

/**
 * Created by zipper on 2/12/17.
 *
 * Demo for judges that shows off our shooting mode
 */
//@Teleop
public class ShootingModeDemo extends FatherboardsLinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        while (!(isStarted() || isStopRequested())) {
            idle();
        }
        waitForStart();
        while(opModeIsActive()) {
            intake();
            shootBall();
        }
    }
}
