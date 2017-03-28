package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by zipper on 3/24/17.
 */
@Autonomous
public class SupportAutoCornerDelayedBlue extends WiredCatsLinearOpMode {
    ElapsedTime shootingTime = new ElapsedTime();
    ElapsedTime mainTime = new ElapsedTime();
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
        telemetry.addData("Initialization ", "complete");
        telemetry.update();
        while (!(isStarted() || isStopRequested())) {
            telemetry();
            idle();
        }
        mainTime.reset();
        while(opModeIsActive() && mainTime.seconds() < 10) {
            idle();
        }
        backwardTicks(.3, 2200, 4);
        sleep(10);
        shootingTime.reset();
        shooterGate.setPosition(.45);
        while(opModeIsActive() && shootingTime.seconds() < 2) {
            flywheelVelocity.setParameters(System.nanoTime(), shooter.getCurrentPosition());
            double currentVelocity = flywheelVelocity.getVelocity();
            telemetry.addData("currentVelocity", currentVelocity);
            velocityBangBang.setParameters(currentVelocity, TARGET_VELOCITY, 0.9, 0.96, TOLERANCE);
            double motorOut = velocityBangBang.getBangBang();
            motorOut = Range.clip(motorOut, 0, 1);
            shooter.setPower(motorOut);
            telemetry.update();
        }
        shootingTime.reset();
        while(opModeIsActive() && shootingTime.seconds() < 6) {
            flywheelVelocity.setParameters(System.nanoTime(), shooter.getCurrentPosition());
            double currentVelocity = flywheelVelocity.getVelocity();
            telemetry.addData("currentVelocity", currentVelocity);
            velocityBangBang.setParameters(currentVelocity, TARGET_VELOCITY, 0.9, 0.96, TOLERANCE);
            double motorOut = velocityBangBang.getBangBang();
            motorOut = Range.clip(motorOut, 0, 1);
            shooter.setPower(motorOut);
            intake.setPower(1);
            telemetry.update();
        }
        shooter.setPower(0);
        intake.setPower(0);
        forwardTicks(.3,1200,3);
        leftTicks(.5,500,3);
        while(opModeIsActive() && mainTime.seconds() < 25) {
            idle();
        }
        backwardTicks(1, 5000, 4);
    }
}
