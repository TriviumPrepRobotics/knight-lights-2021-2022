package org.firstinspires.ftc.teamcode.TuningAndUtility;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Turn Tuning")
//@Disabled
public class TurnTuning extends LinearOpMode {

    DcMotor FrontLeft;
    DcMotor FrontRight;
    DcMotor BackLeft;
    DcMotor BackRight;
    DcMotor Duck;
    DcMotor Slide;
    DcMotor Intake1;
    DcMotor Intake2;

    double ArmPos;
    double ClawPos;

    double leftTriggerStartTime = 0;
    double leftBumperStartTime = 0;
    double rightBumperStartTime = 0;
    double rightTriggerStartTime = 0;
    double bStartTime = 0;
    double yStartTime = 0;
    double aStartTime = 0;
    double xStartTime = 0;

    @Override
    public void runOpMode() throws InterruptedException{

        //INIT STUFFS
        FrontLeft = hardwareMap.dcMotor.get("Front Left");
        FrontRight = hardwareMap.dcMotor.get("Front Right");
        BackLeft = hardwareMap.dcMotor.get("Back Left");
        BackRight = hardwareMap.dcMotor.get("Back Right");
        Duck = hardwareMap.dcMotor.get("Duck");
        Slide = hardwareMap.dcMotor.get("Slide");
        Intake1 = hardwareMap.dcMotor.get("Intake 1");
        Intake2 = hardwareMap.dcMotor.get("Intake 2");

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

        double turnPower = 0.1;

        //READY TO START
        telemetry.addData("Ready to start!", null);
        telemetry.update();

        while(opModeIsActive()) {
            if (gamepad1.right_trigger > 0.5 && gamepad1.left_trigger < 0.5) {
                FrontLeft.setPower(turnPower);
                BackLeft.setPower(turnPower);
                FrontRight.setPower(-turnPower);
                BackRight.setPower(-turnPower);
            }

            if (gamepad1.left_trigger > 0.5 && gamepad1.right_trigger < 0.5) {
                FrontLeft.setPower(-turnPower);
                BackLeft.setPower(-turnPower);
                FrontRight.setPower(turnPower);
                BackRight.setPower(turnPower);
            }

            if (gamepad1.a && aCooldown()) {
                turnPower -= 0.05;
                if (turnPower < -1) {
                    turnPower = -1;
                }
            }

            if (gamepad1.y && yCooldown()) {
                turnPower += 0.05;
                if (turnPower > 1) {
                    turnPower = 1;
                }
            }

            telemetry.addData("Press a to decrease power.", null);
            telemetry.addData("Press y to increase power.", null);
            telemetry.addData("Turnpower: ", turnPower);
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

}
