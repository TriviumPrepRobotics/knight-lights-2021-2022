package org.firstinspires.ftc.teamcode;


import android.graphics.Paint;
import android.provider.FontsContract;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Teleop test", group = "Baby steps")
public class  TestTeleop extends OpMode {

    DcMotor FrontLeft;
    DcMotor FrontRight;
    DcMotor BackLeft;
    DcMotor BackRight;
    DcMotor Duck;
    DcMotor Intake;
    DcMotor Pulley;
    DcMotor Arm;
    Servo Joint;
    Servo Carriage;
    Servo Base;

    public void init() {

        FrontLeft = hardwareMap.dcMotor.get("Front Left");
        FrontRight = hardwareMap.dcMotor.get("Front Right");
        BackLeft = hardwareMap.dcMotor.get("Back Left");
        BackRight = hardwareMap.dcMotor.get("Back Right");
        Duck = hardwareMap.dcMotor.get("Duck");
        Intake = hardwareMap.dcMotor.get("Intake");
        Pulley = hardwareMap.dcMotor.get("Pulley");
        Arm = hardwareMap.dcMotor.get("Arm");
        Joint = hardwareMap.servo.get("Joint");
        Carriage = hardwareMap.servo.get("Carriage");
        Base = hardwareMap.servo.get("Base");

        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Duck.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Pulley.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Pulley.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Base.setPosition(0.69);
        Joint.setPosition(0.1);
        Carriage.setPosition(0.5);
    }


    public void loop() {

        FrontRight.setPower(-gamepad1.right_stick_y * 0.5);
        BackRight.setPower(gamepad1.right_stick_y * 0.5);
        FrontLeft.setPower(gamepad1.left_stick_y * 0.5);
        BackLeft.setPower(-gamepad1.left_stick_y * 0.5);

        Arm.setPower(0);

        Pulley.setPower(0);

        if(gamepad1.x){
            Duck.setPower(0.5);
        }

        if(gamepad1.a){
            Duck.setPower(0);
        }

        if(gamepad1.b){
            Duck.setPower(-0.5);
        }

        if(gamepad1.right_bumper){
            Intake.setPower(0.75);
        }

        if(gamepad1.left_bumper){
            Intake.setPower(0);
        }

        if(gamepad1.dpad_up){
            Pulley.setPower(-0.5);
        }

        if(gamepad1.dpad_down){
            Pulley.setPower(0.25);
        }

        if(gamepad2.dpad_up){
            Base.setPosition(0);
        }

        if(gamepad2.dpad_down){
            Base.setPosition(0.69);
            Joint.setPosition(0.1);
        }

        if(gamepad2.right_bumper){
            Joint.setPosition(0);
        }

        if(gamepad2.left_bumper){
            Joint.setPosition(0.7);
        }

        if(gamepad2.y){
            Arm.setPower(0.75);
        }

        if(gamepad2.a){
            Arm.setPower(-0.2);
        }

        if(gamepad2.x){
            Carriage.setPosition(0.5);
        }

        if(gamepad2.b){
            Carriage.setPosition(0);
        }
    }

}
