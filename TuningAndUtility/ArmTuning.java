package org.firstinspires.ftc.teamcode.TuningAndUtility;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
//@Disabled
public class ArmTuning extends LinearOpMode{

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

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Reached runOpMode", null);
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

        telemetry.addData("Press Play to Start", null);

        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.a) {
                Carriage.setPosition(Carriage.getPosition() + 0.05);
            }
            if(gamepad1.b) {
                Carriage.setPosition(Carriage.getPosition() - 0.05);
            }
            if(gamepad1.x) {
                Door.setPosition(Door.getPosition() + 0.05);
            }
            if(gamepad1.y) {
                Door.setPosition(Door.getPosition() - 0.05);
            }
            telemetry.addData("Carriage Positon: ", Carriage.getPosition());
            telemetry.addData("Door Position: ", Door.getPosition());
        }

    }
}
