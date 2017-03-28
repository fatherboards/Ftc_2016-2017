package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by zipper on 3/23/17.
 */
@Autonomous
public class ShootTwoPark extends WiredCatsLinearOpMode {
    ElapsedTime autoRunTime = new ElapsedTime();
    //Bang Bang Stuff
    private static final double TOLERANCE = 0.5e-7;
    private static final double TARGET_VELOCITY = 2.9e-6;

    protected static final double VELOCITY_P = 7.2;
    protected static final double VELOCITY_I = 0.1;
    protected static final double VELOCITY_D = 0.0;
    VelocityCalculator flywheelVelocity = new VelocityCalculator();
    BangBangCalculator velocityBangBang = new BangBangCalculator();

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        shooterGate.setPosition(0.45);
        while (!(isStarted() || isStopRequested())) {
            telemetry();
            idle();
        }
        autoRunTime.reset();
        encoderDrive(.3, .3, 2100, 2100, 10);
        while(opModeIsActive() && autoRunTime.seconds()<14) {
            flywheelVelocity.setParameters(System.nanoTime(), shooter.getCurrentPosition());
            double currentVelocity = flywheelVelocity.getVelocity();
            telemetry.addData("currentVelocity", currentVelocity);
            velocityBangBang.setParameters(currentVelocity, TARGET_VELOCITY, 0.9, 0.96, TOLERANCE);
            double motorOut = velocityBangBang.getBangBang();
            motorOut = Range.clip(motorOut, 0, 1);
            shooter.setPower(motorOut);
            intake.setPower(1);
        }

        while(opModeIsActive() && autoRunTime.seconds() < 26) {
            intake.setPower(0);
            shooter.setPower(0);
        }
        encoderDrive(.3, .3, 1200, 1200, 10);
    }
}
