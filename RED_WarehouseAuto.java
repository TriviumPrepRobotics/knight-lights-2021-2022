package org.firstinspires.ftc.teamcode;

//Normal FTC Imports

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.Locale;

//Hardware Imports
//Camera Vision Imports
//Sensor Data Interpretation Imports
//Other Imports

@Autonomous(name = "Tournament Red Warehouse", group = "comp")
//@Disabled
public class RED_WarehouseAuto extends LinearOpMode{

    //Hardware Map

    DcMotor FrontRight;
    DcMotor FrontLeft;
    DcMotor BackLeft;
    DcMotor BackRight;

    DcMotor Duck;
    DcMotor Arm;
    DcMotor Claw;

    //Booleans
    boolean duckOn = false;
    boolean clawOn = false;

    double power = 0.5;


    @Override
    public void runOpMode() throws InterruptedException{

        //Hardware Map Stuffs
        FrontLeft = hardwareMap.dcMotor.get("Front Left");
        FrontRight = hardwareMap.dcMotor.get("Front Right");
        BackLeft = hardwareMap.dcMotor.get("Back Left");
        BackRight = hardwareMap.dcMotor.get("Back Right");
        Duck = hardwareMap.dcMotor.get("Duck");
        Arm = hardwareMap.dcMotor.get("Arm");
        Claw = hardwareMap.dcMotor.get("Claw");

        BackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontRight.setDirection(DcMotorSimple.Direction.FORWARD);


        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Claw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Claw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        //Finished Initialization

        waitForStart();

        moveForward(18);
        sleep(250);
        EncoderTurn(-10);
        sleep(250);
        moveForward(5);
        sleep(250);
        armTop();
        sleep(500);
        clawOn();
        sleep(250);
        clawOn();
        sleep(250);
        armBack();
        sleep(250);
        EncoderTurn(-10);
        sleep(250);
        Speed(-60);

    }

    //Movement Methods

    public void moveBackward (double distance){ //537.7 PPR
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double circumference = 3.14 * 5; //pi times wheel diameter
        double rotationsneeded = distance / circumference; //length(in) divided by circumference
        int encoderdrivingtarget = (int) (rotationsneeded * 538); //rotations needed times motor ticks per revolution

        FrontLeft.setTargetPosition(-encoderdrivingtarget);
        FrontRight.setTargetPosition(-encoderdrivingtarget);
        BackLeft.setTargetPosition(-encoderdrivingtarget);
        BackRight.setTargetPosition(-encoderdrivingtarget);

        FrontLeft.setPower(.4);
        FrontRight.setPower(.4);
        BackLeft.setPower(.4);
        BackRight.setPower(.4);

        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(FrontRight.isBusy() || FrontLeft.isBusy()) {

        }

        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
    }

    public void moveForward (double distance) { //537.7 PPR
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double circumference = 3.14 * 5; //pi times wheel diameter
        double rotationsneeded = distance / circumference; //length(in) divided by circumference
        int encoderdrivingtarget = (int) (rotationsneeded * 538); //rotations needed times motor ticks per revolution

        FrontLeft.setTargetPosition(encoderdrivingtarget);
        FrontRight.setTargetPosition(encoderdrivingtarget);
        BackLeft.setTargetPosition(encoderdrivingtarget);
        BackRight.setTargetPosition(encoderdrivingtarget);

        FrontLeft.setPower(.4);
        FrontRight.setPower(.4);
        BackLeft.setPower(.4);
        BackRight.setPower(.4);

        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(FrontRight.isBusy() || FrontLeft.isBusy()) {

        }

        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
    }

    public void Speed (double distance) { //537.7 PPR
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double circumference = 3.14 * 5; //pi times wheel diameter
        double rotationsneeded = distance / circumference; //length(in) divided by circumference
        int encoderdrivingtarget = (int) (rotationsneeded * 538); //rotations needed times motor ticks per revolution

        FrontLeft.setTargetPosition(encoderdrivingtarget);
        FrontRight.setTargetPosition(encoderdrivingtarget);
        BackLeft.setTargetPosition(encoderdrivingtarget);
        BackRight.setTargetPosition(encoderdrivingtarget);

        FrontLeft.setPower(1);
        FrontRight.setPower(1);
        BackLeft.setPower(1);
        BackRight.setPower(1);

        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(FrontRight.isBusy() || FrontLeft.isBusy()) {

        }

        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
    }

    public void EncoderTurn(double distance) {
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double circumference = 3.14 * 5; //pi times wheel diameter
        double rotationsneeded = distance / circumference; //length(in) divided by circumference
        int encoderdrivingtarget = (int) (rotationsneeded * 538); //rotations needed times motor ticks per revolution

        FrontLeft.setTargetPosition(encoderdrivingtarget);
        FrontRight.setTargetPosition(-encoderdrivingtarget);
        BackLeft.setTargetPosition(encoderdrivingtarget);
        BackRight.setTargetPosition(-encoderdrivingtarget);

        FrontLeft.setPower(.2);
        FrontRight.setPower(.2);
        BackLeft.setPower(.2);
        BackRight.setPower(.2);

        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(FrontRight.isBusy() || FrontLeft.isBusy()) {

        }

        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
    }

    public void armTop() {
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setTargetPosition(590);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm.setPower(0.20);
        while(Arm.isBusy()){}
    }

    public void armBack() {
        Arm.setTargetPosition(350);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm.setPower(0.20);
        while(Arm.isBusy()){}
    }

    public void duckSwitch() {
        duckOn = !duckOn;
        if (duckOn) {
            Duck.setPower(-0.5);
        } else {
            Duck.setPower(0);
        }
    }

    public void clawOn() {
        clawOn = !clawOn;
        if(clawOn){
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
