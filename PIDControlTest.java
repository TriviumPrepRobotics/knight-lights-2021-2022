package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

@Autonomous
//@Disabled
public class PIDControlTest extends LinearOpMode{

    DcMotor FrontLeft;
    DcMotor FrontRight;
    DcMotor BackLeft;
    DcMotor BackRight;

    //The IMU unit object
    BNO055IMU imu;

    // State used for updating telemetry (whatever that means)
    Orientation angles;
    Acceleration gravity;

    double gain = 1;
    double power = 0.5;

    double maxRotationalAcceleration;

    @Override
    public void runOpMode() throws InterruptedException {
        FrontLeft = hardwareMap.dcMotor.get("Front Left");
        FrontRight = hardwareMap.dcMotor.get("Front Right");
        BackLeft = hardwareMap.dcMotor.get("Back Left");
        BackRight = hardwareMap.dcMotor.get("Back Right");

        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);

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

        while (opModeIsActive()) {
            telemetry.update();
            turnLeft(angles.firstAngle, 45);
        }

    }

    //Proportional Check
    void proportionalCheck(int currentHeading, int setHeading) {
        //Error = Setpoint - ProcessingValue;
        //Output = K * Error;
        double error = currentHeading - setHeading;
        double output = gain * error;

    }

    //Integral Check
    double reset = 0;
    void integralCheck(int currentHeading, int setHeading) {
        //Error = Setpoint - ProcessingValue
        //Reset = Reset + K/tau_i * Error
        //Output = K * Error + Reset
        double error = currentHeading - setHeading;
        reset = reset + gain/6.28 * error;
        double output = gain * error + reset;
    }

    //Derivative Check
    double lastError = 0;
    void derivativeCheck(int currentHeading, int setHeading) {
        //Error = setPoint - processingValue
        //Output = K * Error + K/tau_i * (Error - LastError)
        //LastError = Error
        double error = currentHeading - setHeading;
        double output  = gain * error + gain/6.28 * (error - lastError);
        lastError = error;
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

    //Physics Equations
    double pastPosition = 0;
    double pastVelTime = 0;

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

    double powerCoefficient() {
        power = 0.3;
        double value = (1 - (angles.firstAngle / 180)) / 0.007;
        return (power += value);
    }

    void turnLeft(double originHeading, double turnHeading) {
        double targetHeading = originHeading + turnHeading;
        double halfWay = targetHeading - (turnHeading / 2);
        double interval = (targetHeading - originHeading) / 100;
        while(angles.firstAngle < halfWay) {
            telemetry.update();
            power = 0.4 + 0.006 * ((angles.firstAngle - originHeading) / interval);
            FrontLeft.setPower(-power);
            BackLeft.setPower(-power);
            FrontRight.setPower(power);
            BackRight.setPower(power);
        }
        while(angles.firstAngle > halfWay && angles.firstAngle < targetHeading) {
            telemetry.update();
            power = 0.4 + 0.006 * ((targetHeading - angles.firstAngle) / interval);
            FrontLeft.setPower(-power);
            BackLeft.setPower(-power);
            FrontRight.setPower(power);
            BackRight.setPower(power);
        }
        power = 0;
    }

}
