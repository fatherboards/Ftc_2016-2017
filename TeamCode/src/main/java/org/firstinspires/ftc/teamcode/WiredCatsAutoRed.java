package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by zipper on 3/23/17.
 */
@Autonomous
public class WiredCatsAutoRed extends WiredCatsLinearOpMode {
    ElapsedTime shootingTime = new ElapsedTime();

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
        leftButtonPusher.setPosition(1.0);
        rightButtonPusher.setPosition(0);
        sleep(300);
        leftButtonPusher.setPosition(0.0);
        rightButtonPusher.setPosition(1);
        sleep(50);
        backwardTicks(.3, 1700, 4);
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
        while(opModeIsActive() && shootingTime.seconds() < 3) {
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
        forwardTicks(.3, 700, 4);
        sleep(10);
        leftTicks(.5, 1300, 3);
        sleep(10);
        forwardTicks(0.3, 3500, 4);
        sleep(10);
        leftTicks(.5, 400, 3);
        sleep(10);
        forwardTicks(0.3, 4000, 6);
        //encoderDrive(.3,.5,4000, 4000,6);
        sleep(10);
        while(opModeIsActive()) {
            setPower(-.25,-.12);
            if(getColor(beaconColorRight).equals("red")) {
                setPower(-.2,-.2);
                rightButtonPusher.setPosition(.75);
                shootingTime.reset();
                while(opModeIsActive() && shootingTime.seconds() < 3) {
                    setPower(-.20,-.20);
                }
                rightButtonPusher.setPosition(1);
            }
        }
    }
}
