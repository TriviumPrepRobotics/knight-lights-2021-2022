package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Red TeleOp", group = "comp")
//@Disabled
public class WillRed extends LinearOpMode{

    DcMotor FrontLeft;
    DcMotor FrontRight;
    DcMotor BackLeft;
    DcMotor BackRight;
    DcMotor Duck;
    DcMotor Slide;
    DcMotor Intake1;
    DcMotor Intake2;

    Servo servo;

    double leftTriggerStartTime = 0;
    double leftBumperStartTime = 0;
    double rightBumperStartTime = 0;
    double rightTriggerStartTime = 0;
    double bStartTime = 0;
    double yStartTime = 0;
    double aStartTime = 0;
    double xStartTime = 0;
    double power = 0.5;

    boolean duckOn = false;
    boolean ClawOn = false;
    boolean turboMode = false;
    boolean pickup = false;
    boolean deliver = false;
    boolean carry = false;
    boolean intakeOn;


    @Override
    public void runOpMode() throws InterruptedException{

        FrontLeft = hardwareMap.dcMotor.get("Front Left");
        FrontRight = hardwareMap.dcMotor.get("Front Right");
        BackLeft = hardwareMap.dcMotor.get("Back Left");
        BackRight = hardwareMap.dcMotor.get("Back Right");
        Duck = hardwareMap.dcMotor.get("Duck");
        Slide = hardwareMap.dcMotor.get("Slide");
        Intake1 = hardwareMap.dcMotor.get("Intake 1");
        Intake2 = hardwareMap.dcMotor.get("Intake 2");

        servo = hardwareMap.servo.get("servo");

        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Duck.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servo.setPosition(1);

        waitForStart();

        while(opModeIsActive()){

            if(gamepad1.left_bumper){
                turboMode = !turboMode;
            }

            if (turboMode) {
                FrontLeft.setPower(gamepad1.left_stick_y);
                FrontRight.setPower(-gamepad1.right_stick_y);
                BackLeft.setPower(-gamepad1.left_stick_y);
                BackRight.setPower(gamepad1.right_stick_y);
            } else {
                FrontLeft.setPower(gamepad1.left_stick_y * power);
                FrontRight.setPower(-gamepad1.right_stick_y * power);
                BackLeft.setPower(-gamepad1.left_stick_y * power);
                BackRight.setPower(gamepad1.right_stick_y * power);
            }

            if(gamepad1.a && aCooldown()){
                duckSwitch();
            }

            if(gamepad1.b){
                pickup = true;
                deliver = false;
                carry = false;
                servo.setPosition(1);
                Slide.setTargetPosition(0);
                Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Slide.setPower(1);
                intakeOn = false;
            }

            if(gamepad1.x){
                deliver = false;
                pickup = false;
                carry = true;
                servo.setPosition(1);
                Slide.setTargetPosition(965);
                Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Slide.setPower(1);
                intakeOn = false;
            }

            if(gamepad1.y){
                carry = false;
                pickup = false;
                deliver = true;
                servo.setPosition(0);
                Slide.setTargetPosition(4368);
                Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Slide.setPower(1);
                intakeOn = false;
            }

            if(pickup) {
                if(gamepad1.right_bumper && rightBumperCooldown()){ intakeSwitch2(); }
            } else if(deliver){
                if(gamepad1.right_bumper && rightBumperCooldown()) { intakeSwitch1(); }
            } else {
                if(gamepad1.right_bumper && rightBumperCooldown()) { intakeSwitch1(); }
            }

            Slide.setPower(-1);

            telemetry.addData("pickup", pickup);
            telemetry.addData("carry", carry);
            telemetry.addData("deliver", deliver);
            telemetry.update();

        }

    }

    public boolean leftTriggerCooldown() {
        if(getRuntime() - leftTriggerStartTime > 0.25) { //Must wait 250 milliseconds before input can be used again
            leftTriggerStartTime = getRuntime();
            return true;
        }
        return false;
    }

    public boolean leftBumperCooldown() {
        if(getRuntime() - leftBumperStartTime > 0.25) { //Must wait 250 milliseconds before input can be used again
            leftBumperStartTime = getRuntime();
            return true;
        }
        return false;
    }

    public boolean rightTriggerCooldown() {
        if(getRuntime() - rightTriggerStartTime > 0.25) { //Must wait 250 milliseconds before input can be used again
            rightTriggerStartTime = getRuntime();
            return true;
        }
        return false;
    }

    public boolean rightBumperCooldown() {
        if(getRuntime() - rightBumperStartTime > 0.25) { //Must wait 250 milliseconds before input can be used again
            rightBumperStartTime = getRuntime();
            return true;
        }
        return false;
    }

    public boolean xCooldown() {
        if(getRuntime() - xStartTime > .25) { //Must wait 250 milliseconds before input can be used again
            xStartTime = getRuntime();
            return true;
        }
        return false;
    }

    public boolean bCooldown() {
        if(getRuntime() - bStartTime > .25) { //Must wait 250 milliseconds before input can be used again
            bStartTime = getRuntime();
            return true;
        }
        return false;
    }

    public boolean yCooldown() {
        if(getRuntime() - yStartTime > .25) { //Must wait 250 milliseconds before input can be used again
            yStartTime = getRuntime();
            return true;
        }
        return false;
    }

    public boolean aCooldown() {
        if(getRuntime() - aStartTime > .25) { //Must wait 250 milliseconds before input can be used again
            aStartTime = getRuntime();
            return true;
        }
        return false;
    }

    public void duckSwitch() {
        duckOn = !duckOn;
        if (duckOn) {
            Duck.setPower(-0.5);
        } else {
            Duck.setPower(0);
        }
    }

    public void intakeSwitch1() {
        intakeOn = !intakeOn;
        if(intakeOn){
            Intake1.setPower(0.5);
            Intake2.setPower(-0.5);
        } else {
            Intake1.setPower(0);
            Intake2.setPower(0);
        }
    }

    public void intakeSwitch2() {
        intakeOn = !intakeOn;
        if(intakeOn){
            Intake1.setPower(-0.5);
            Intake2.setPower(0.5);
        } else {
            Intake1.setPower(0);
            Intake2.setPower(0);
        }
    }

}