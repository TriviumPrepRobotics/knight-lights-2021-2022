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

@Autonomous(name = "Tournament Blue Warehouse", group = "comp")
//@Disabled
public class BLUE_WarehouseAuto extends LinearOpMode{

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

    BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;

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

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        composeTelemetry();

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        //Finished Initialization

        waitForStart();

        moveForward(15);
        sleep(250);
        turn(-40);
        sleep(250);
        moveForward(9);
        sleep(250);
        armTop();
        sleep(500);
        clawOn();
        sleep(250);
        clawOn();
        sleep(250);
        armBack();
        sleep(250);
        turn(-85);
        sleep(250);
        Speed(-72);

    }

    double acceptableErrorMargin = 1;
    double powerCoefficient = 0.7;
    double basePower = 0.4;

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

        FrontLeft.setTargetPosition(-encoderdrivingtarget);
        FrontRight.setTargetPosition(encoderdrivingtarget);
        BackLeft.setTargetPosition(-encoderdrivingtarget);
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

    void turn(double target) {
        //SETS MOTORS TO BE ABLE TO RUN WITHOUT ENCODER INPUTS
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //TAKES INTIIAL ANGLE GROUP AND SETS ORIGIN TO THE Z ANGLE
        angles = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentPos = angles.firstAngle;
        boolean turnRight;

        while (!(calculateError(target, currentPos) < acceptableErrorMargin)) {
            angles = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentPos = angles.firstAngle;

            turnRight = directionCheck(target, currentPos);
            twiddle(target, currentPos, turnRight);
        }

    }

    void twiddle(double target, double origin, boolean turnRight) {

        double currentPos = origin;

        double turnPower = /*basePower * ((calculateError(target, currentPos))/30) + basePower; */ 1;

        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int i = 0;

        if (turnRight) {
            FrontLeft.setPower(turnPower);
            FrontRight.setPower(-turnPower);
            BackLeft.setPower(turnPower);
            BackRight.setPower(-turnPower);

            while (!(currentPos > target) && i % 5 == 0) {
                angles = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                currentPos = angles.firstAngle;
                i++;
            }

            FrontLeft.setPower(0);
            FrontRight.setPower(0);
            BackLeft.setPower(0);
            BackRight.setPower(0);
        }

        if (!turnRight) {
            FrontLeft.setPower(-turnPower);
            FrontRight.setPower(turnPower);
            BackLeft.setPower(-turnPower);
            BackRight.setPower(turnPower);

            while (!(currentPos < target) && i % 5 == 0) {
                angles = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                currentPos = angles.firstAngle;
                i++;
            }

            FrontLeft.setPower(0);
            FrontRight.setPower(0);
            BackLeft.setPower(0);
            BackRight.setPower(0);
        }
    }

    double calculateError(double target, double origin) {
        if (target > origin && origin >= 0) {
            return target - origin;
        }
        if (target > origin && origin <= 0) {
            return target + Math.abs(origin);
        }
        if (origin > target && target >= 0) {
            return origin - target;
        }
        if (origin > target && target <= 0) {
            return origin + Math.abs(target);
        }
        return 0;
    }

    boolean directionCheck(double target, double origin) {
        if (target > origin) {
            return false;
        }
        return true;
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

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}
