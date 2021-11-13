package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
//@Disabled
public class DanielBlue extends LinearOpMode{

    DcMotor FrontLeft;
    DcMotor FrontRight;
    DcMotor BackLeft;
    DcMotor BackRight;

    DcMotor Duck;
    DcMotor Intake;
    DcMotor Pulley;
    DcMotor Arm;

    Servo Door;
    Servo Joint;
    Servo Carriage;

    double power = 0.5; //Power Coefficient determines power to the wheel motors of robot
    boolean turboMode = false; //Turbomode puts the robot at 100% power in the wheel motors

    boolean intakeOn = false;
    boolean pulleyUp = false;
    boolean duckOn = false;
    boolean doorOpen = false;

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

        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);

        Duck = hardwareMap.dcMotor.get("Duck");
        Intake = hardwareMap.dcMotor.get("Intake");
        Pulley = hardwareMap.dcMotor.get("Pulley");
        Arm = hardwareMap.dcMotor.get("Arm");

        Joint = hardwareMap.servo.get("Joint");
        Carriage = hardwareMap.servo.get("Carriage");
        Door = hardwareMap.servo.get("Door");

        //Initialization Movement
        pulleySwitch(); //Lifts Intake
        pulleyUp = false; //Resets Intake Variable
        Joint.setPosition(0.69); //Sets Joint Servo Position
        Carriage.setPosition(0.1); //Sets Carriage Servo Position
        Door.setPosition(0.5); //Sets Door Servo Position

        Pulley.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //Sets Pulley to ZeroPowerBehavior
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //Sets Arm to ZeroPowerBehavior

        waitForStart();

        while (opModeIsActive()) {
            defaultMode();
            telemetry.addData("Runtime: ", getRuntime());
            telemetry.addData("Pulley Boolean: ", pulleyUp);
            telemetry.update();
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
        if (gamepad1.left_bumper && leftBumperCooldown()) {
            intakeSwitch();
        }

        //Pulley Up/Down
        if (gamepad1.left_trigger > 0.5 && leftTriggerCooldown()) {
            pulleySwitch();
        }

        //Duck Wheel On/Off
        if (gamepad1.b && bCooldown()) {
            duckSwitch();
        }

        //Move Arm to Position 1
        if (gamepad1.y && yCooldown()) {
            armMoveMid();
        }

        //Move Arm to Position 2
        if (gamepad1.a && aCooldown()) {
            armMoveShort();
        }

        if(gamepad1.x && xCooldown()) {
            armCollapse();
        }

        if(gamepad1.right_bumper && rightBumperCooldown()) {
            doorSwitch();
        }

    }

    //Input Cooldowns
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

    //Hold Check Methods
    public boolean holdCheck() {
        return true;
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
            Pulley.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            Pulley.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Pulley.setTargetPosition(-376);
            Pulley.setPower(1);
            Pulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (Pulley.isBusy()) {
                defaultMode();
            }
            Pulley.setPower(0);
            Pulley.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

    public void doorSwitch() {
        doorOpen = !doorOpen;
        if(doorOpen = true) {
            Door.setPosition(0);
        } else {
            Door.setPosition(0.5);
        }
    }

    public void armCollapse() {
        //get Encoder Target
        int encoderArmTarget = -(Arm.getCurrentPosition());

        Joint.setPosition(0.69); //Joint Servo is at 0.69
        Carriage.setPosition(0.1); //Carriage Servo is at 0.1

        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setTargetPosition(encoderArmTarget); //Arm Encoder is at the horizontal
        Arm.setPower(1);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Arm.isBusy()) {
            defaultMode();
        }
        Arm.setPower(0);
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void armMoveShort() {
        //Get Encoder Target
        int encoderArmTarget = -(Arm.getCurrentPosition());

        Joint.setPosition(0.05); //Joint Servo is at 0.05
        Carriage.setPosition(0.1); //Carriage Servo is at 0.1

        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setTargetPosition(encoderArmTarget); //Arm Encoder is 45 degrees above horizontal (encoder ticks)
        Arm.setPower(1);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Arm.isBusy()) {
            defaultMode();
        }
        Arm.setPower(0);
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void armMoveMid() {
        //Get Encoder Target
        int encoderArmTarget = 0;
        if(Arm.getCurrentPosition() > 100) {
            encoderArmTarget = 100 - Arm.getCurrentPosition();
        } else if (Arm.getCurrentPosition() < 100) {
            encoderArmTarget = -(Arm.getCurrentPosition() - 100);
        }

        Joint.setPosition(0.02); //Joint Servo is at 0
        Carriage.setPosition(0.05); //Carriage Servo is at 0.05

        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setTargetPosition(encoderArmTarget); //Arm Encoder is 18 degrees above horizontal (encoder ticks)
        Arm.setPower(1);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Arm.isBusy()) {
            defaultMode();
        }
        Arm.setPower(0);
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void armMoveHigh() {
        //Get Encoder Target
        int encoderArmTarget = 0;
        if(Arm.getCurrentPosition() > 250) {
            encoderArmTarget = 250 - Arm.getCurrentPosition();
        } else if (Arm.getCurrentPosition() < 100) {
            encoderArmTarget = -(Arm.getCurrentPosition() - 250);
        }

        Joint.setPosition(0.02); //Joint Servo is at 0
        Carriage.setPosition(0.02); //Carriage Servo is at 0

        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setTargetPosition(encoderArmTarget); //Arm Encoder is 45 degrees above horizontal (encoder ticks)
        Arm.setPower(1);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Arm.isBusy()) {
            defaultMode();
        }
        Arm.setPower(0);
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

}
