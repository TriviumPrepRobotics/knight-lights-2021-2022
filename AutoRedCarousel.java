package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

@Autonomous(name = "RedCarousel", group = "")
//@Disabled
public class AutoRedCarousel extends LinearOpMode {

    //Hardware Map
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

    //Switch Booleans
    boolean intakeOn = false;
    boolean pulleyUp = false;
    boolean duckOn = false;
    boolean doorOpen = false;
    boolean isMid = false;
    boolean isShort = false;
    boolean isCollapsed = true;

    //CV Stuffs
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {"Ball", "Cube", "Duck", "Marker"};

    private static final String VUFORIA_KEY =
            "AR/sArn/////AAABmYT4OHM7O0ghuzdm7dDNDFhZG6Yl6O3GSKrhCfuGTcO9wTvGG646aPxMUSyLSgqkS7u8oRDM8K744VEXHCOYpsfzxT7Gp/Evu8537krH3m91cimF8TITI5nvIsA9bOXSqInL1u+X0uGgU9ZA44A0Ox4nJy/2Yi7YhlPrPl9A30bgrkY+/azNr1SGVGucWZQHACG9/6NkO5KhyWL5ImeiNEsE10mIqvA9FkcC8iWA5ANwGmtsGkzgW01HgYy43BIP2zULL9Bq44vzCda7agK03HvYqgAasu3D8KSI3ecm5E+hvJiFIe62ptFx728jErgtOr5gyKV5oo8nO60x74XUXuu1kpptLjwxXHQOk0h4CHFc";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    //IMU Stuffs
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    double power;

    @Override
    public void runOpMode() {

        //Hardware Map
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

        //CV Stuffs
        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
        }

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        //IMU Stuffs
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

        waitForStart();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        if (opModeIsActive()) {
            telemetry.update();
            moveForward(30);
            turnRight(angles.firstAngle, 90);
            /*TFOD stuff for when we need it
        tfod.getUpdatedRecognitions();
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if(updatedRecognitions != null){
            for(Recognition recognition : updatedRecognitions){
                if(recognition.getLeft() > 200 && recognition.getRight() > 200){
                   //left code here
                }
                if(recognition.getLeft() < 200 && recognition.getRight() < 200){
                    //mid code here
                }
            }
        }
        else{
            //right code here
        }
            */
        }
        }

        private void initVuforia () {
            /*
             * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
             */
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

            parameters.vuforiaLicenseKey = VUFORIA_KEY;
            parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

            //  Instantiate the Vuforia engine
            vuforia = ClassFactory.getInstance().createVuforia(parameters);

            // Loading trackables is not necessary for the TensorFlow Object Detection engine.
        }

        private void initTfod () {
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfodParameters.minResultConfidence = 0.8f;
            tfodParameters.isModelTensorFlow2 = true;
            tfodParameters.inputSize = 320;
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        }

        public void pulleySwitch () {
            pulleyUp = !pulleyUp;
            if (pulleyUp == true) {
                Pulley.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Pulley.setTargetPosition(376);
                Pulley.setPower(1);
                Pulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (Pulley.isBusy()) {
                }
                Pulley.setPower(0);
                Pulley.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else {
                Pulley.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Pulley.setTargetPosition(-376);
                Pulley.setPower(1);
                Pulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (Pulley.isBusy()) {
                }
                Pulley.setPower(0);
                Pulley.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }

        public void moveForward ( double distance){ //537.7 PPR
            FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            double circumference = 3.14 * 5; //pi times wheel diameter
            double rotationsneeded = 12 / circumference; //length(in) divided by circumference
            int encoderdrivingtarget = (int) (rotationsneeded * 538); //rotations needed times motor ticks per revolution

            //encoderdrivingtarget = 1028

            FrontLeft.setTargetPosition(encoderdrivingtarget);
            FrontRight.setTargetPosition(encoderdrivingtarget);
            BackLeft.setTargetPosition(encoderdrivingtarget);
            BackRight.setTargetPosition(encoderdrivingtarget);


            FrontLeft.setPower(.6);
            FrontRight.setPower(.6);
            BackLeft.setPower(.6);
            BackRight.setPower(.6);

            FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    void turnLeft(double originHeading, double turnHeading) {
        double targetHeading = originHeading + turnHeading;
        double halfWay = targetHeading - (turnHeading / 2);
        double interval = (targetHeading - originHeading) / 10;
        while(angles.firstAngle < halfWay) {
            telemetry.addData("First Half: ", true);
            telemetry.addData("Second Half: ", false);
            telemetry.addData("Target Heading: ", targetHeading);
            telemetry.update();
            power = 0.3;
            //power = 0.2 + (powerMax - 0.2) * ((angles.firstAngle - originHeading) / interval);
            //power = 0.25 + 0.05 * ((angles.firstAngle - originHeading) / interval);
            FrontLeft.setPower(power);
            BackLeft.setPower(power);
            FrontRight.setPower(-power);
            BackRight.setPower(-power);
        }
        while(angles.firstAngle > halfWay && angles.firstAngle < targetHeading) {
            telemetry.addData("Second Half: ", true);
            telemetry.addData("First Half: ", false);
            telemetry.addData("Target Heading: ", targetHeading);
            telemetry.update();
            //power = 0.2 + (powerMax - 0.2) * ((targetHeading - angles.firstAngle) / interval);
            power = 0.2 + 0.05 * ((targetHeading - angles.firstAngle) / interval);
            FrontLeft.setPower(power);
            BackLeft.setPower(power);
            FrontRight.setPower(-power);
            BackRight.setPower(-power);
        }
        //Correction Code
        telemetry.addData("Second Half: ", false);
        telemetry.addData("First Half: ", false);
        telemetry.update();
        power = 0;
    }

    void turnRight(double originHeading, double turnHeading) {
        double targetHeading = originHeading - turnHeading;
        double halfWay = targetHeading + (turnHeading / 2);
        double interval = -(targetHeading + originHeading) / 10;
        while(angles.firstAngle > halfWay) {
            telemetry.addData("First Half: ", true);
            telemetry.addData("Second Half: ", false);
            telemetry.addData("Target Heading: ", targetHeading);
            telemetry.update();
            power = -0.3;
            //power = 0.2 + (powerMax - 0.2) * ((angles.firstAngle - originHeading) / interval);
            //power = 0.25 + 0.05 * ((angles.firstAngle - originHeading) / interval);
            FrontLeft.setPower(power);
            BackLeft.setPower(power);
            FrontRight.setPower(-power);
            BackRight.setPower(-power);
        }
        while(angles.firstAngle < halfWay && angles.firstAngle > targetHeading) {
            telemetry.addData("Second Half: ", true);
            telemetry.addData("First Half: ", false);
            telemetry.addData("Target Heading: ", targetHeading);
            telemetry.update();
            //power = 0.2 + (powerMax - 0.2) * ((targetHeading - angles.firstAngle) / interval);
            power = -(0.2 + 0.05 * ((targetHeading - angles.firstAngle) / interval));
            FrontLeft.setPower(power);
            BackLeft.setPower(power);
            FrontRight.setPower(-power);
            BackRight.setPower(-power);
        }
        //Correction Code
        telemetry.addData("Second Half: ", false);
        telemetry.addData("First Half: ", false);
        telemetry.update();
        power = 0;
    }

    }
