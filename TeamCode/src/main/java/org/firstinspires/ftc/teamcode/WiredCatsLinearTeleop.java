package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by STUDENT on 3/17/2017.
 */
@TeleOp(name = "WiredCatsLinearTeleop", group = "Teleop")
public class WiredCatsLinearTeleop extends WiredCatsLinearOpMode {
    /**
     * You declare variables like a toddler
     *
     * Hehe XD
     * ~Zipzop
     *
     * HEHE XD I am a Toddler
     *
     * ~Gwong
     */


    //Counters
    int intakeToggle = 0;
    int outtakeToggle = 0;
    int shooterToggle = 0;
    int leftButtonPusherToggle = 0;
    int rightButtonPusherToggle = 0;
    int releaseToggle = 0;
    int frontPusherToggle = 0;
    int shooterGateToggle = 0;
    int shootingModeToggle = 0;

    //Bang Bang Stuff
    private static final double TOLERANCE = 0.5e-7;
    private static final double TARGET_VELOCITY = 2.9e-6;

    protected static final double VELOCITY_P = 7.2;
    protected static final double VELOCITY_I = 0.1;
    protected static final double VELOCITY_D = 0.0;
    VelocityCalculator flywheelVelocity = new VelocityCalculator();
    BangBangCalculator velocityBangBang = new BangBangCalculator();

    //Shooting Mode Stuff
    boolean inShootingMode = false;
    ElapsedTime shootingModeTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        while(!isStarted()){
            idle();
        }
        while(opModeIsActive()) {
            doTeleop();
            if (gamepad2.a && shootingModeToggle == 0){

                shootingModeToggle = 1;

            }
            else if (!gamepad2.a && shootingModeToggle == 1){
                shootingModeTime.reset();
                inShootingMode = true;
                while(opModeIsActive() && shootingModeTime.seconds() < 2.5) {
                    doTeleop();
                    flywheelVelocity.setParameters(System.nanoTime(), shooter.getCurrentPosition());
                    double currentVelocity = flywheelVelocity.getVelocity();
                    telemetry.addData("currentVelocity", currentVelocity);
                    velocityBangBang.setParameters(currentVelocity, TARGET_VELOCITY, 0.9, 0.96, TOLERANCE);
                    double motorOut = velocityBangBang.getBangBang();
                    motorOut = Range.clip(motorOut, 0, 1);
                    shooter.setPower(motorOut);
                    telemetry.update();
                }
                shootingModeToggle = 2;

            }
            shootingModeTime.reset();
            while(opModeIsActive()&& inShootingMode) {
                doTeleop();
                if(Math.round(shootingModeTime.seconds())%3==0) {
                    shooterGate.setPosition(0);
                }
                else {
                    shooterGate.setPosition(.45);
                }
                flywheelVelocity.setParameters(System.nanoTime(), shooter.getCurrentPosition());
                double currentVelocity = flywheelVelocity.getVelocity();
                telemetry.addData("currentVelocity", currentVelocity);
                velocityBangBang.setParameters(currentVelocity, TARGET_VELOCITY, 0.9, 0.96, TOLERANCE);
                double motorOut = velocityBangBang.getBangBang();
                motorOut = Range.clip(motorOut, 0, 1);
                shooter.setPower(motorOut);
                shooterGate.setPosition(.45);
                intake.setPower(1);
                if (gamepad2.a && shootingModeToggle == 2) {
                    shootingModeToggle = 3;

                } else if (!gamepad2.a && shootingModeToggle == 3) {
                    inShootingMode = false;
                    shooter.setPower(0);
                    intake.setPower(0);
                    shooterGate.setPosition(0);
                    shootingModeToggle = 0;

                }
                telemetry.update();
            }
        }
    }
    public void doTeleop() {
        //Drive Train
        float lefty = gamepad1.left_stick_y;
        float rightx = gamepad1.right_stick_x;

        if (lefty > 0.05 || lefty < -0.05 || rightx > 0.05 || rightx < -0.05){

            leftFront.setPower(lefty + -rightx);
            rightFront.setPower(lefty + rightx);
            leftBack.setPower(lefty + -rightx);
            rightBack.setPower(lefty + rightx);

        }
        else{

            leftFront.setPower(0.0);
            rightFront.setPower(0.0);
            leftBack.setPower(0.0);
            rightBack.setPower(0.0);

        }

        //Intake
        if (gamepad1.a && intakeToggle == 0){

            intakeToggle = 1;

        }
        else if (!gamepad1.a && intakeToggle == 1){

            intake.setPower(1.0);
            intakeToggle = 2;

        }
        else if (gamepad1.a && intakeToggle == 2){

            intakeToggle = 3;

        }
        else if (!gamepad1.a && intakeToggle == 3){

            intake.setPower(0.0);
            intakeToggle = 0;

        }

        if (gamepad1.b && outtakeToggle == 0){

            outtakeToggle = 1;

        }
        else if (!gamepad1.b && outtakeToggle == 1){

            intake.setPower(-1.0);
            outtakeToggle = 2;

        }
        else if (gamepad1.b && outtakeToggle == 2){

            outtakeToggle = 3;

        }
        else if (!gamepad1.b && outtakeToggle == 3){

            intake.setPower(0.0);
            outtakeToggle = 0;

        }

        //Side Pushers

        boolean leftBumper = gamepad1.left_bumper;
        boolean rightBumper = gamepad1.right_bumper;

        if (leftBumper && leftButtonPusherToggle == 0){

            leftButtonPusherToggle = 1;

        }
        else if (!leftBumper && leftButtonPusherToggle == 1){

            leftButtonPusher.setPosition(0.2);
            leftButtonPusherToggle = 2;

        }
        else if (leftBumper && leftButtonPusherToggle == 2){

            leftButtonPusherToggle = 3;

        }
        else if (!leftBumper && leftButtonPusherToggle == 3){

            leftButtonPusher.setPosition(0);
            leftButtonPusherToggle = 0;

        }

        if (rightBumper && rightButtonPusherToggle == 0){

            rightButtonPusherToggle = 1;

        }
        else if (!rightBumper && rightButtonPusherToggle == 1){

            rightButtonPusher.setPosition(0.8);
            rightButtonPusherToggle = 2;

        }
        else if (rightBumper && rightButtonPusherToggle == 2){

            rightButtonPusherToggle = 3;

        }
        else if (!rightBumper && rightButtonPusherToggle == 3){

            rightButtonPusher.setPosition(1);
            rightButtonPusherToggle = 0;

        }

        //Release
        if (gamepad2.b && releaseToggle == 0){

            releaseToggle = 1;

        }
        else if (!gamepad2.b && releaseToggle == 1){

            release.setPosition(0.0);
            releaseToggle = 2;

        }
        else if (gamepad2.b && releaseToggle == 2){

            releaseToggle = 3;

        }
        else if (!gamepad2.b && releaseToggle == 3){

            release.setPosition(0.5);
            releaseToggle = 0;

        }

        //Front Pusher
        if (gamepad1.x && frontPusherToggle == 0){

            frontPusherToggle = 1;

        }
        else if (!gamepad1.x && frontPusherToggle == 1){

            leftFrontPusher.setPosition(0);
            rightFrontPusher.setPosition(1);
            frontPusherToggle = 2;

        }
        else if (gamepad1.x && frontPusherToggle == 2){

            frontPusherToggle = 3;

        }
        else if (!gamepad1.x && frontPusherToggle == 3){

            leftFrontPusher.setPosition(1);
            rightFrontPusher.setPosition(0);
            frontPusherToggle = 0;

        }

        //Winch
        float operatorLeftStick = gamepad2.left_stick_y;
        float operatorRightStick = gamepad2.right_stick_y;

        float operatorLeftTrigger = gamepad2.left_trigger;
        float operatorRightTrigger = gamepad2.right_trigger;

        if (operatorLeftTrigger > 0.05 && operatorRightTrigger < 0.05){

            winchleft.setPower(operatorLeftTrigger);
            winchright.setPower(operatorLeftTrigger);

        }
        else if (operatorLeftTrigger < 0.05 && operatorRightTrigger > 0.05){

            winchleft.setPower(-operatorRightTrigger);
            winchright.setPower(-operatorRightTrigger);

        }
        else if (operatorLeftTrigger < 0.05 && operatorRightTrigger < 0.05){

            winchleft.setPower(operatorLeftStick);
            winchright.setPower(operatorRightStick);

        }
        else if (gamepad1.y){

            winchleft.setPower(-0.5);
            winchright.setPower(-0.5);

        }
        else{

            winchleft.setPower(0.0);
            winchright.setPower(0.0);

        }

    }
}
