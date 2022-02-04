package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Red TeleOp", group = "comp")
//@Disabled
public class WillRed extends LinearOpMode{

    DcMotor FrontLeft;
    DcMotor FrontRight;
    DcMotor BackLeft;
    DcMotor BackRight;
    DcMotor Duck;
    DcMotor Arm;
    DcMotor Claw;

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


    @Override
    public void runOpMode() throws InterruptedException{

        FrontLeft = hardwareMap.dcMotor.get("Front Left");
        FrontRight = hardwareMap.dcMotor.get("Front Right");
        BackLeft = hardwareMap.dcMotor.get("Back Left");
        BackRight = hardwareMap.dcMotor.get("Back Right");
        Duck = hardwareMap.dcMotor.get("Duck");
        Arm = hardwareMap.dcMotor.get("Arm");
        Claw = hardwareMap.dcMotor.get("Claw");

        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Claw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Claw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while(Arm.getCurrentPosition() != 0){
            idle();
        }

        while(Claw.getCurrentPosition() != 0){
            idle();
        }

        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Duck.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Claw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

            if(gamepad1.x){
                Arm.setTargetPosition(350);
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm.setPower(0.35);
            }

            if(gamepad1.dpad_up){
                Arm.setTargetPosition(590);
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm.setPower(0.35);
            }

            if(gamepad1.dpad_left){
                Arm.setTargetPosition(725);
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm.setPower(0.35);
            }

            if(gamepad1.dpad_down){
                Arm.setTargetPosition(825);
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm.setPower(0.35);
            }

            if(gamepad1.b){
                Arm.setTargetPosition(900);
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm.setPower(0.35);
            }

            if(gamepad1.right_bumper && rightBumperCooldown()){
                ClawOn();
            }

            if(Arm.getCurrentPosition() < - 500){
                Arm.setPower(0.15);
            } else {
                Arm.setPower(0.35);
            }

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

    public void ClawOn() {
        ClawOn = !ClawOn;
        if(ClawOn){
            Claw.setTargetPosition(300);
            Claw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Claw.setPower(0.25);
        } else {
            Claw.setTargetPosition(0);
            Claw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Claw.setPower(0.25);
        }
    }

}