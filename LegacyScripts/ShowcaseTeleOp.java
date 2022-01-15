package org.firstinspires.ftc.teamcode.LegacyScripts;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
//@Disabled
public class ShowcaseTeleOp extends LinearOpMode{

    DcMotor FrontLeft;
    DcMotor FrontRight;
    DcMotor BackLeft;
    DcMotor BackRight;

    DcMotor Launch;
    DcMotor WobbleArm;
    DcMotor Intake;
    DcMotor Compliant;

    Servo LaunchServo;
    Servo IntakeServo;
    Servo WobbleServo;

    double Power = 0.7;
    int WobbleServoCounter = 2;
    int IntakeServoCounter = 2;
    int LaunchServoCounter = 2;
    int LaunchWheelCounter = 2;

    @Override
    public void runOpMode() throws InterruptedException {
        FrontLeft = hardwareMap.dcMotor.get("Front Left");
        FrontRight = hardwareMap.dcMotor.get("Front Right");
        BackLeft = hardwareMap.dcMotor.get("Back Left");
        BackRight = hardwareMap.dcMotor.get("Back Right");
        Launch = hardwareMap.dcMotor.get("Launch");
        WobbleArm = hardwareMap.dcMotor.get("Wobble");
        Intake = hardwareMap.dcMotor.get("Intake");
        Compliant = hardwareMap.dcMotor.get("Compliant");

        LaunchServo = hardwareMap.servo.get("Launch Servo");
        IntakeServo = hardwareMap.servo.get("Intake Servo");
        WobbleServo = hardwareMap.servo.get("Wobble Servo");

        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);

        WobbleServo.setPosition(0.3); //Close on Init
        IntakeServo.setPosition(0.2); //Close on Init
        LaunchServo.setPosition(0.6); //CLose on Init


        waitForStart();

        while(opModeIsActive()){
            //CHASSIS MOVEMENT
            String Current = "Stopped";
            /* Strafing Stuffs */
            /* Strafe Right (Right Trigger) */
            if (Current.equals("Stopped")) {
                //Current = "Strafe";
                FrontLeft.setPower(-gamepad1.right_trigger/Power);
                FrontRight.setPower(gamepad1.right_trigger/Power);
                BackLeft.setPower(gamepad1.right_trigger/Power);
                BackRight.setPower(-gamepad1.right_trigger/Power);
                Current = "Stopped";
            }
            /* Strafe Left (Left Trigger)  */
            if (Current.equals("Stopped")) {
                //Current = "Strafe";
                FrontLeft.setPower(gamepad1.left_trigger/Power);
                FrontRight.setPower(-gamepad1.left_trigger/Power);
                BackLeft.setPower(-gamepad1.left_trigger/Power);
                BackRight.setPower(gamepad1.left_trigger/Power);
                Current = "Stopped";
            }
            /* Basic Movement (Tank Style) */
            if (Current.equals("Stopped")) {
                //Current = "Basic Drive";
                if (gamepad1.right_bumper){
                    FrontLeft.setPower(gamepad1.left_stick_y);
                    FrontRight.setPower(gamepad1.right_stick_y);
                    BackLeft.setPower(gamepad1.left_stick_y);
                    BackRight.setPower(gamepad1.right_stick_y);
                } else {
                    FrontLeft.setPower(gamepad1.left_stick_y / Power);
                    FrontRight.setPower(gamepad1.right_stick_y / Power);
                    BackLeft.setPower(gamepad1.left_stick_y / Power);
                    BackRight.setPower(gamepad1.right_stick_y / Power);
                }
                //Current = "Stopped";
            }

            telemetry.addData("FrontLeft Power", FrontLeft.getPower());
            telemetry.addData("FrontRight Power", FrontRight.getPower());
            telemetry.addData("BackLeft Power", BackLeft.getPower());
            telemetry.addData("BackRight Power", BackRight.getPower());
            telemetry.update();

            //Wobble Movement
            if (gamepad1.left_bumper) {
                WobbleOut();
            }

            if (gamepad1.right_bumper) {
                WobbleServo();
            }

            if (gamepad1.dpad_up) {
                WobbleUp();
            }

            if (gamepad1.dpad_down) {
                WobbleDown();
            }
            //Intake Servo
            if (gamepad1.dpad_right) {
                if((IntakeServoCounter%2) == 0) {
                    IntakeServo.setPosition(0); //Open
                    IntakeServoCounter += 1;
                } else if ((IntakeServoCounter%2) == 1) {
                    IntakeServo.setPosition(0.2); //Close
                    IntakeServoCounter += 1;
                }
            }
            //Intake Control
            if (gamepad1.b) {
                Intake.setPower(1);
            }

            if (gamepad1.a) {
                Intake.setPower(-1);
            }

            if (gamepad1.dpad_left) {
                Intake.setPower(0);
            }
            //Launch Control
            if (gamepad1.y) {
                if((LaunchServoCounter%2) == 0) {
                    LaunchServo.setPosition(4); //Open
                    LaunchServoCounter += 1;
                } else if ((LaunchServoCounter%2) == 1) {
                    LaunchServo.setPosition(0.6); //Close
                    LaunchServoCounter += 1;
                }
            }

            if (gamepad1.x) {
                if((LaunchWheelCounter%2) == 0) {
                    Launch.setPower(1);
                    Compliant.setPower(1);
                    LaunchWheelCounter += 1;
                } else if ((LaunchWheelCounter%2) == 1) {
                    Launch.setPower(0);
                    Compliant.setPower(0);
                    LaunchWheelCounter += 1;
                }
            }

        }

    }

    //Wobble Arm Methods
    private void WobbleOut() {
        WobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WobbleArm.setTargetPosition(900);
        WobbleArm.setPower(-0.35);
        WobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(WobbleArm.isBusy()) {
            if(gamepad1.x) {
                WobbleArm.setPower(0);
                WobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }
        WobbleArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void WobbleIn() {
        WobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WobbleArm.setTargetPosition(-900);
        WobbleArm.setPower(0.35);
        WobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(WobbleArm.isBusy()) {
            if(gamepad1.x) {
                WobbleArm.setPower(0);
                WobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }
        WobbleArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void WobbleUp() {
        WobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WobbleArm.setTargetPosition(-300);
        WobbleArm.setPower(0.35);
        WobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(WobbleArm.isBusy()) {
            if(gamepad1.x) {
                WobbleArm.setPower(0);
                WobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }
        WobbleArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void WobbleDown() {
        WobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WobbleArm.setTargetPosition(300);
        WobbleArm.setPower(0.35);
        WobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(WobbleArm.isBusy()) {
            if(gamepad1.x) {
                WobbleArm.setPower(0);
                WobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }
        WobbleArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void WobbleServo() {

        if((WobbleServoCounter%2) == 0) {
            WobbleServo.setPosition(0); //Open
            WobbleServoCounter += 1;
        } else if ((WobbleServoCounter%2) == 1) {
            WobbleServo.setPosition(0.3); //Close
            WobbleServoCounter += 1;
        }

    }
}
