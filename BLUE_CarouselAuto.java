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

@Autonomous(name = "Tournament Blue Carousel", group = "comp")
//@Disabled
public class BLUE_CarouselAuto extends LinearOpMode{

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

    //Coefficients for PD Control
    double proportionalCoefficient = 0.7;
    double derivativeCoefficient = 0.7;

    //Camera Vision Stuffs
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };

    private static final String VUFORIA_KEY = "AR/sArn/////AAABmYT4OHM7O0ghuzdm7dDNDFhZG6Yl6O3GSKrhCfuGTcO9wTvGG646aPxMUSyLSgqkS7u8oRDM8K744VEXHCOYpsfzxT7Gp/Evu8537krH3m91cimF8TITI5nvIsA9bOXSqInL1u+X0uGgU9ZA44A0Ox4nJy/2Yi7YhlPrPl9A30bgrkY+/azNr1SGVGucWZQHACG9/6NkO5KhyWL5ImeiNEsE10mIqvA9FkcC8iWA5ANwGmtsGkzgW01HgYy43BIP2zULL9Bq44vzCda7agK03HvYqgAasu3D8KSI3ecm5E+hvJiFIe62ptFx728jErgtOr5gyKV5oo8nO60x74XUXuu1kpptLjwxXHQOk0h4CHFc";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    //IMU Stuffs
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

        //Vuforia Stuffs
//        initVuforia();

        //TensorFlow Stuffs
//        initTfod();
/*        if (tfod != null) {
            tfod.activate();
            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2.5, 16.0/9.0);
        }

        //Scan for element
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
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
                }
                telemetry.update();
            }
        }
*/
        //IMU Stuffs
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        composeTelemetry();

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        //Finished Initialization

        waitForStart();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
/*
        moveForward(6);
        turn(90);
        moveBackward(25);
        duckSwitch();
        sleep(2000);
        duckSwitch();
        moveForward(6);
        telemetry.addData("Turn started", null);
        telemetry.update();
        Realturn(45);
        telemetry.addData("Turn finished", null);
        telemetry.update();
        moveForward(40);
        armTop();
        sleep(250);
        clawOn();
        sleep(250);
        armBack();
        moveBackward(20);
        turn(90);
        moveForward(80);
 */
        moveForward(18);
        sleep(500);
        Realturn(45);
        moveForward(6);
        sleep(500);
        armTop();
        sleep(500);
        clawOn();
        sleep(500);
        clawOn();
        sleep(500);
        armBack();

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

    void turn(double targetAngle) {
        double origin = angles.firstAngle;
        double target = targetAngle;

        if (target < origin) {
            FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            FrontLeft.setPower(-0.6);
            BackLeft.setPower(-0.6);
            FrontRight.setPower(0.6);
            BackRight.setPower(0.6);

            while(FrontRight.isBusy() || BackLeft.isBusy() || FrontLeft.isBusy() || BackRight.isBusy()) {
                if (angles.firstAngle <= target + 10) {
                    FrontLeft.setPower(0);
                    BackLeft.setPower(0);
                    FrontRight.setPower(0);
                    BackRight.setPower(0);
                }
            }
        }

        if (target > origin) {
            FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            FrontLeft.setPower(0.6);
            BackLeft.setPower(0.6);
            FrontRight.setPower(-0.6);
            BackRight.setPower(-0.6);

            while(FrontRight.isBusy() || BackLeft.isBusy() || FrontLeft.isBusy() || BackRight.isBusy()) {
                if (angles.firstAngle >= target - 10) {
                    FrontLeft.setPower(0);
                    BackLeft.setPower(0);
                    FrontRight.setPower(0);
                    BackRight.setPower(0);
                }
            }
        }

    }

    public void EncoderTurn(double distance) {
        telemetry.addData("turning", null);
        telemetry.update();
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double circumference = 3.14 * 5;
        /*
        double target = distance / 360;
        double encodertarget = target * 51.7;
        */
        double rotationsneeded = distance / circumference;
        int encoderdrivingtarget = (int) (rotationsneeded * 538);
        telemetry.addData("math done", null);
        telemetry.update();

        FrontLeft.setTargetPosition(encoderdrivingtarget);
        FrontRight.setTargetPosition(-encoderdrivingtarget);
        BackLeft.setTargetPosition(encoderdrivingtarget);
        BackRight.setTargetPosition(-encoderdrivingtarget);
        telemetry.addData("encoder set", null);
        telemetry.update();

        FrontLeft.setPower(0.5);
        FrontRight.setPower(0.5);
        BackLeft.setPower(0.5);
        BackRight.setPower(0.5);

        sleep(1000);

        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
    }

    void Realturn(double target) {
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        boolean finished = false;

        angles = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double origin = angles.firstAngle;
        telemetry.addData("Can read from firstAngle: ", origin);
        telemetry.update();
        boolean turnRight = directionCheck(target, origin);
        telemetry.addData("turnRight: ", turnRight);
        telemetry.update();
        double distance = calculateTurnDistance(target, origin);
        telemetry.addData("calculate turn distance: ", distance);
        telemetry.update();

        double turnPower = proportionalCoefficient * (distance/360) + 0.3;
        double dturnPower = derivativeCoefficient * (distance/360) + 0.3;

        double halfway = calculateHalfway(target, angles.firstAngle, distance);
        telemetry.addData("calculate turn halfway: ", halfway);
        telemetry.update();
        beginAcceleration(turnPower, distance, turnRight);
        telemetry.addData("begin moving: ", true);
        telemetry.update();
        int counter = 1;
        while (!finished) {
            if (counter % 25 == 0 || counter == 1) {
                angles = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                telemetry.addData("Data can be seen here!", counter);
                telemetry.addData("Current Heading", angles.firstAngle);
                telemetry.update();
            }
            /*
            telemetry.update();
            if (halfwayCheck(halfway, turnRight)) {
                beginDecceleration(dturnPower, distance, turnRight);
                telemetry.addData("slow down: ", true);
                telemetry.update();
            }
            */

            if (turnRight) {
                if (angles.firstAngle <= target + 5.0) {
                    telemetry.addData("has reached target: ", true);
                    telemetry.update();
                    FrontRight.setPower(0);
                    FrontLeft.setPower(0);
                    BackLeft.setPower(0);
                    BackRight.setPower(0);
                    finished = true;
                }
            } else {
                if (angles.firstAngle >= target - 5.0) {
                    telemetry.addData("has reached target: ", true);
                    telemetry.update();
                    FrontRight.setPower(0);
                    FrontLeft.setPower(0);
                    BackLeft.setPower(0);
                    BackRight.setPower(0);
                    finished = true;
                }
            }
            counter++;
        }

    }

    double pastPosition = 0;
    double pastVelTime = 0;

    //System Methods

    public void armTop() {
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setTargetPosition(-900);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm.setPower(0.35);
        while(Arm.isBusy()){}
    }

    public void armBack() {
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setTargetPosition(-350);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm.setPower(0.35);
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

    double calculateAngularVelocity() {
        double currentPosition = (double) angles.firstAngle;
        double currentTime = getRuntime();
        double angularVelocity = (currentPosition - pastPosition) / (currentTime - pastVelTime);
        pastPosition = currentPosition;
        pastVelTime = currentTime;
        return angularVelocity;
    }

    double pastVelocity = 0;
    double pastAccTime = 0;

    double calculateAngularAcceleration() {
        double currentVelocity = calculateAngularVelocity();
        double currentTime = getRuntime();
        double angularAcceleration = (currentVelocity - pastVelocity) / (currentTime - pastAccTime);
        pastVelocity = currentVelocity;
        pastAccTime = currentTime;
        return angularAcceleration;
    }

    boolean directionCheck(double target, double origin) {
        if (target > origin) {
            return false;
        }
        return true;
    }

    double calculateTurnDistance(double target, double origin) {
        if (target > origin && origin >= 0) {
            return target - origin;
        }
        if (target > origin && origin < 0) {
            return target + Math.abs(origin);
        }
        if (origin > target && target >= 0) {
            return origin - target;
        }
        if (origin > target && target < 0) {
            return origin + Math.abs(target);
        }
        return 0;
    }

    double calculateHalfway(double target, double origin, double distance) {
        if (target > origin) {
            return target - (distance/2);
        }
        if (origin > target) { // && target >= 0 <--- Removed this from the conditional parameters
            return origin - (distance/2);
        }
        return 0;
    }

    void beginAcceleration(double gain, double distance, boolean turnRight) {
        double turnPower = gain * (distance/360) + 0.3;

        if (turnRight) {
            FrontLeft.setPower(turnPower);
            BackLeft.setPower(turnPower);
            FrontRight.setPower(-turnPower);
            BackRight.setPower(-turnPower);
        } else {
            FrontLeft.setPower(-turnPower);
            BackLeft.setPower(-turnPower);
            FrontRight.setPower(turnPower);
            BackRight.setPower(turnPower);
        }

    }

    void beginDecceleration(double gain, double distance, boolean turnRight) {
        double turnPower = gain * (distance/360) + 0.3;

        if (turnRight) {
            FrontLeft.setPower(-turnPower);
            BackLeft.setPower(-turnPower);
            FrontRight.setPower(turnPower);
            BackRight.setPower(turnPower);
        } else {
            FrontLeft.setPower(turnPower);
            BackLeft.setPower(turnPower);
            FrontRight.setPower(-turnPower);
            BackRight.setPower(-turnPower);
        }

    }

    boolean halfwayCheck(double halfway, boolean turnRight) {
        if (turnRight) {
            if (angles.firstAngle < halfway) {
                return true;
            }
            return false;
        }
        if (angles.firstAngle > halfway) {
            return true;
        }
        return false;
    }

    //Vuforia Methods

    private void initVuforia() {
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

    //TensorFlow Methods

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    //Telemetry Format Methods

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

}
