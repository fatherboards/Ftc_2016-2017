package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by zipper on 2/12/17.
 */
@TeleOp
public class BalanceDemo extends FatherboardsLinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        while (!(isStarted() || isStopRequested())) {
            idle();
        }
        waitForStart();
        balance();
    }
}
