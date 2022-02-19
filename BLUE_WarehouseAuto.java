package org.firstinspires.ftc.teamcode;

//Normal FTC Imports

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

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
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.Locale;

//Hardware Imports
//Camera Vision Imports
//Sensor Data Interpretation Imports
//Other Imports

@Autonomous(name = "Tournament Blue Warehouse", group = "comp")
//@Disabled
public class BLUE_WarehouseAuto extends LinearOpMode{

    //Hardware Map
    DcMotor FrontLeft;
    DcMotor FrontRight;
    DcMotor BackLeft;
    DcMotor BackRight;
    DcMotor Duck;
    DcMotor Slide;
    DcMotor Intake1;
    DcMotor Intake2;

    Servo servo;

    //Booleans
    boolean duckOn = false;
    boolean ClawOn = false;
    boolean turboMode = false;
    boolean pickup = false;
    boolean deliver = false;
    boolean carry = false;
    boolean intakeOn;

    double power = 0.5;

    BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;

    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {"Ball", "Cube", "Duck", "Marker"};

    private static final String VUFORIA_KEY =
            "AR/sArn/////AAABmYT4OHM7O0ghuzdm7dDNDFhZG6Yl6O3GSKrhCfuGTcO9wTvGG646aPxMUSyLSgqkS7u8oRDM8K744VEXHCOYpsfzxT7Gp/Evu8537krH3m91cimF8TITI5nvIsA9bOXSqInL1u+X0uGgU9ZA44A0Ox4nJy/2Yi7YhlPrPl9A30bgrkY+/azNr1SGVGucWZQHACG9/6NkO5KhyWL5ImeiNEsE10mIqvA9FkcC8iWA5ANwGmtsGkzgW01HgYy43BIP2zULL9Bq44vzCda7agK03HvYqgAasu3D8KSI3ecm5E+hvJiFIe62ptFx728jErgtOr5gyKV5oo8nO60x74XUXuu1kpptLjwxXHQOk0h4CHFc";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    @Override
    public void runOpMode() throws InterruptedException{

        //Hardware Map Stuffs
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

        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servo.setPosition(1);

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

        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1, 16.0/9.0);
        }

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        //Finished Initialization

        waitForStart();

        //OBJECT LOCATION STATUSES
        boolean right = false;
        boolean left = false;

        //THIS VALUE NEEDS TO BE CALIBRATED TO BE IN THE CENTER OF THE FRAME. JUST CHECK THIS WITH TFOD WHEN LOOKING AT A DUCK AND LINE THE LEFT COORDINATES OF THE DUCK WHERE WE WANT THE BOUNDARY
        float boundary = 200;

        if(tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                    i++;

                    String duckstring = recognition.getLabel();
                    //'IF' STATEMENT CHECKS IF THE OBJECT IS ON THE RIGHT OR LEFT OF THE FRAME. DO NOT USE MORE THAN ONE DUCK WHEN TESTING.
                    if (duckstring.equalsIgnoreCase("Duck") && recognition.getLeft() < boundary) {
                        left = true;
                        right = false;
                    } else if (duckstring.equalsIgnoreCase("Duck") && recognition.getLeft() > boundary) {
                        right = true;
                        left = false;
                    }
                }
            }
        }

        carry();
        sleep(500);
        moveForward(18);
        sleep(500);
        turn(-30);
        sleep(500);

        if(right == true) {
            deliver();
            sleep(1000);
            moveForward(10);
            sleep(500);
            output();
            sleep(500);
            moveBackward(6);
            sleep(500);
            carry();
        } else if(left == true){
            mid();
            sleep(1000);
            moveForward(9);
            sleep(500);
            output();
            sleep(500);
            moveBackward(5);
            sleep(500);
            carry();
        } else {
            bottom();
            sleep(500);
            moveForward(9);
            sleep(500);
            output();
            sleep(500);
            moveBackward(5);
            sleep(500);
            carry();
        }
        sleep(500);
        turn(-90);
        sleep(500);
        Speed(-54);
        sleep(500);

        pickup();



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

    public void deliver() {
        carry = false;
        pickup = false;
        deliver = true;
        servo.setPosition(0);
        Slide.setTargetPosition(4050);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slide.setPower(1);
        intakeOn = false;
    }

    public void mid() {
        carry = false;
        pickup = false;
        deliver = true;
        servo.setPosition(0);
        Slide.setTargetPosition(2631);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slide.setPower(1);
        intakeOn = false;
    }

    public void bottom() {
        carry = false;
        pickup = false;
        deliver = true;
        servo.setPosition(1);
        Slide.setTargetPosition(994);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slide.setPower(1);
        intakeOn = false;
    }

    public void carry() {
        deliver = false;
        pickup = false;
        carry = true;
        servo.setPosition(1);
        Slide.setTargetPosition(940);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slide.setPower(1);
        intakeOn = false;
    }

    public void pickup() {
        pickup = true;
        deliver = false;
        carry = false;
        servo.setPosition(1);
        Slide.setTargetPosition(0);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slide.setPower(1);
        intakeOn = false;
        while(Slide.isBusy()){}
    }

    public void output() {
        Intake1.setPower(0.5);
        Intake2.setPower(-0.5);
        sleep(1000);
        Intake1.setPower(0);
        Intake2.setPower(0);
    }

    public void intake() {
        Intake1.setPower(-0.5);
        Intake2.setPower(0.5);
        sleep(1000);
        Intake1.setPower(0);
        Intake2.setPower(0);
    }

    public void duckSwitch() {
        duckOn = !duckOn;
        if (duckOn) {
            Duck.setPower(-0.5);
        } else {
            Duck.setPower(0);
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

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.65f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
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
