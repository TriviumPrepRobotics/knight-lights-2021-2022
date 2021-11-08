package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Disabled
public class DanielBlue extends LinearOpMode{

    DcMotor FrontLeft;
    DcMotor FrontRight;
    DcMotor BackLeft;
    DcMotor BackRight;

    DcMotor Duck;
    DcMotor Intake;
    DcMotor Pulley;
    DcMotor Arm;

    Servo Joint1;
    Servo Joint2;
    Servo Carriage;

    double power = 0.5; //Power Coefficient determines power to the wheel motors of robot
    boolean turboMode = false; //Turbomode puts the robot at 100% power in the wheel motors

    boolean intakeOn = false;
    boolean pulleyUp = false;
    boolean duckOn = false;

    double leftTriggerStartTime = 0;
    double leftBumperStartTime = 0;
    double rightBumperStartTime = 0;
    double rightTriggerStartTime = 0;
    double bStartTime = 0;
    double yStartTime = 0;
    double aStartTime = 0;
    double xStartTime = 0;
    
    @Override
    public void runOpMode() {
        FrontLeft = hardwareMap.dcMotor.get("Front Left");
        FrontRight = hardwareMap.dcMotor.get("Front Right");
        BackLeft = hardwareMap.dcMotor.get("Back Left");
        BackRight = hardwareMap.dcMotor.get("Back Right");

        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);

        Duck = hardwareMap.dcMotor.get("Duck");
        Intake = hardwareMap.dcMotor.get("Intake");
        Pulley = hardwareMap.dcMotor.get("Pulley");
        Arm = hardwareMap.dcMotor.get("Arm");

        waitForStart();

        while (opModeIsActive()) {
            defaultMode();
        }
    }

    public void defaultMode() {
        //Switches full speed on and off
        if (gamepad1.right_bumper) {
            turboMode = !turboMode;
        }

        //Tank Drive Movement
        if (turboMode) {
            FrontLeft.setPower(gamepad1.left_stick_y);
            FrontRight.setPower(gamepad1.right_stick_y);
            BackLeft.setPower(gamepad1.left_stick_y);
            BackRight.setPower(gamepad1.right_stick_y);
        } else {
            FrontLeft.setPower(gamepad1.left_stick_y * power);
            FrontRight.setPower(gamepad1.right_stick_y * power);
            BackLeft.setPower(gamepad1.left_stick_y * power);
            BackRight.setPower(gamepad1.right_stick_y * power);
        }

        //Intake On/Off
        if (gamepad1.left_bumper && leftBumperCheck()) {
            intakeSwitch();
        }

        //Pulley Up/Down
        if (gamepad1.left_trigger > 0.5 && leftTriggerCheck()) {
            pulleySwitch();
        }

        //Duck Wheel On/Off
        if (gamepad1.b && bCheck()) {
            duckSwitch();
        }

        //Move Arm to Position 1
        if (gamepad1.y) {

        }

        //Move Arm to Position 2
        if (gamepad1.a) {

        }

    }

    //Input Cooldowns
    public boolean leftTriggerCheck() {
        if(getRuntime() - leftTriggerStartTime > 250) { //Must wait 250 milliseconds before input can be used again
            leftTriggerStartTime = getRuntime();
            return true;
        }
        return false;
    }

    public boolean leftBumperCheck() {
        if(getRuntime() - leftBumperStartTime > 250) { //Must wait 250 milliseconds before input can be used again
            leftBumperStartTime = getRuntime();
            return true;
        }
        return false;
    }

    public boolean rightTriggerCheck() {
        if(getRuntime() - rightTriggerStartTime > 250) { //Must wait 250 milliseconds before input can be used again
            rightTriggerStartTime = getRuntime();
            return true;
        }
        return false;
    }

    public boolean rightBumperCheck() {
        if(getRuntime() - rightBumperStartTime > 250) { //Must wait 250 milliseconds before input can be used again
            rightBumperStartTime = getRuntime();
            return true;
        }
        return false;
    }

    public boolean bCheck() {
        if(getRuntime() - bStartTime > 250) { //Must wait 250 milliseconds before input can be used again
            bStartTime = getRuntime();
            return true;
        }
        return false;
    }

    public boolean yCheck() {
        if(getRuntime() - yStartTime > 250) { //Must wait 250 milliseconds before input can be used again
            yStartTime = getRuntime();
            return true;
        }
        return false;
    }

    public boolean aCheck() {
        if(getRuntime() - aStartTime > 250) { //Must wait 250 milliseconds before input can be used again
            aStartTime = getRuntime();
            return true;
        }
        return false;
    }

    //System Methods
    public void pulleySwitch() {
        pulleyUp = !pulleyUp;
        if (pulleyUp == true) {
            Pulley.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Pulley.setTargetPosition(376);
            Pulley.setPower(1);
            Pulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (Pulley.isBusy()) {
                defaultMode();
            }
            Pulley.setPower(0);
        } else {
            Pulley.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Pulley.setTargetPosition(376);
            Pulley.setPower(-1);
            Pulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (Pulley.isBusy()) {
                defaultMode();
            }
            Pulley.setPower(0);
        }
    }

    public void intakeSwitch() {
        intakeOn = !intakeOn;
        if (intakeOn) {
            Intake.setPower(1);
        } else {
            Intake.setPower(0);
        }
    }

    public void duckSwitch() {
        duckOn = !duckOn;
        if (duckOn) {
            Duck.setPower(1);
        } else {
            Duck.setPower(0);
        }
    }

}
