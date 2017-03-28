package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by STUDENT on 3/17/2017.
 */
@TeleOp(name = "WiredCatsTeleop", group = "Teleop")
public class tele9110 extends OpMode {
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
    //Drive Train
    DcMotor frontleft;
    DcMotor frontright;
    DcMotor backleft;
    DcMotor backright;

    //Shooter
    DcMotor intake;
    DcMotor shooter;

    //Winch
    DcMotor winchleft;
    DcMotor winchright;

    //Side Pushers
    Servo leftButtonPusher;
    Servo rightButtonPusher;

    //Release
    Servo release;

    //FrontPusher
    Servo leftFrontPusher;
    Servo rightFrontPusher;

    //Gate
    Servo shooterGate;

    //Counters
    int intakeToggle = 0;
    int outtakeToggle = 0;
    int shooterToggle = 0;
    int leftButtonPusherToggle = 0;
    int rightButtonPusherToggle = 0;
    int releaseToggle = 0;
    int frontPusherToggle = 0;
    int shooterGateToggle = 0;

    //Bang Bang Stuff
    private static final double TOLERANCE = 0.5e-7;
    private static final double TARGET_VELOCITY = 2.9e-6;

    protected static final double VELOCITY_P = 7.2;
    protected static final double VELOCITY_I = 0.1;
    protected static final double VELOCITY_D = 0.0;
    VelocityCalculator flywheelVelocity = new VelocityCalculator();
    BangBangCalculator velocityBangBang = new BangBangCalculator();

    @Override
    public void init(){

        frontleft = hardwareMap.dcMotor.get("fl");
        frontright = hardwareMap.dcMotor.get("fr");
        backleft = hardwareMap.dcMotor.get("bl");
        backright = hardwareMap.dcMotor.get("br");

        intake = hardwareMap.dcMotor.get("int");
        shooter = hardwareMap.dcMotor.get("s");

        winchleft = hardwareMap.dcMotor.get("wl");
        winchright = hardwareMap.dcMotor.get("wr");

        leftButtonPusher = hardwareMap.servo.get("leftButtonPusher");
        rightButtonPusher = hardwareMap.servo.get("rightButtonPusher");

        release = hardwareMap.servo.get("capDropper");

        leftFrontPusher = hardwareMap.servo.get("leftFrontPusher");
        rightFrontPusher = hardwareMap.servo.get("rightFrontPusher");

        shooterGate = hardwareMap.servo.get("shooterGate");

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        winchright.setDirection(DcMotorSimple.Direction.REVERSE);

        leftButtonPusher.setPosition(0);
        rightButtonPusher.setPosition(1);
        release.setPosition(0.5);
        leftFrontPusher.setPosition(0);
        rightFrontPusher.setPosition(1);
        shooterGate.setPosition(0);


    }
    @Override
    public void loop(){

        //Drive Train
        float lefty = gamepad1.left_stick_y;
        float rightx = gamepad1.right_stick_x;

        if (lefty > 0.05 || lefty < -0.05 || rightx > 0.05 || rightx < -0.05){

            frontleft.setPower(lefty + -rightx);
            frontright.setPower(lefty + rightx);
            backleft.setPower(lefty + -rightx);
            backright.setPower(lefty + rightx);

        }
        else{

            frontleft.setPower(0.0);
            frontright.setPower(0.0);
            backleft.setPower(0.0);
            backright.setPower(0.0);

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

        //Shooter
        if (gamepad2.a && shooterToggle == 0){

            shooterToggle = 1;

        }
        else if (!gamepad2.a && shooterToggle == 1){

            flywheelVelocity.setParameters(System.nanoTime(), shooter.getCurrentPosition());
            double currentVelocity = flywheelVelocity.getVelocity();
            telemetry.addData("currentVelocity", currentVelocity);
            velocityBangBang.setParameters(currentVelocity, TARGET_VELOCITY, 0.9, 0.96, TOLERANCE);
            double motorOut = velocityBangBang.getBangBang();
            motorOut = Range.clip(motorOut, 0, 1);
            shooter.setPower(motorOut);
            shooterToggle = 2;

        }
        else if (gamepad2.a && shooterToggle == 2){

            shooterToggle = 3;

        }
        else if (!gamepad2.a && shooterToggle == 3){

            shooter.setPower(0);
            shooterToggle = 0;

        }

        if (gamepad2.y && shooterGateToggle == 0){

            shooterGateToggle = 1;

        }
        else if (!gamepad2.y && shooterGateToggle == 1){

            shooterGate.setPosition(0.45);
            shooterGateToggle = 2;

        }
        else if (gamepad2.y && shooterGateToggle == 2){

            shooterGateToggle = 3;

        }
        else if (!gamepad2.y && shooterGateToggle == 3){

            shooterGate.setPosition(0);
            shooterGateToggle = 0;

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
        telemetry.addData("shooterGateDown", shooterGateToggle==2 || shooterGateToggle==3);
        telemetry.update();
    }

}
class BangBangCalculator
{
    private double target = 0;
    private double velocity = 0;
    private double lowerPower, higherPower = 0;
    private double tolerance = 0;

    public void setParameters(double target, double velocity, double lowerPower, double higherPower, double tolerance)
    {
        this.target = target;
        this.velocity = velocity;
        this.lowerPower = lowerPower;
        this.higherPower = higherPower;
        this.tolerance = tolerance;
    }

    public double getBangBang()
    {
        if(velocity >= (target + tolerance))
        {
            return lowerPower;
        }

        else
        {
            return higherPower;
        }
    }
}
class VelocityCalculator
{
    private long time, encoder = 0;

    private long lastEncoder, lastTime = 0;

    public void setParameters(long time, long encoder)
    {
        this.time = time;
        this.encoder = encoder;
    }

    public double getVelocity()
    {
        double velocity = (double) (encoder - lastEncoder) / (time - lastTime);

        lastEncoder = encoder;
        lastTime = time;

        return velocity;
    }
}
