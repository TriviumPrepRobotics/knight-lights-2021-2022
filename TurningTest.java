package org.firstinspires.ftc.teamcode.miscellaneous;

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

import java.util.ArrayList;

@Autonomous
//@Disabled
public class TurningTest extends LinearOpMode{

    DcMotor FrontLeft;
    DcMotor FrontRight;
    DcMotor BackLeft;
    DcMotor BackRight;

    //The IMU unit object
    BNO055IMU imu;

    // State used for updating telemetry (whatever that means)
    Orientation angles;
    Acceleration gravity;

    //Coefficients for PD Control
    double proportionalCoefficient = 0.7;
    double derivativeCoefficient = 0.7;

    @Override
    public void runOpMode() throws InterruptedException {
        FrontLeft = hardwareMap.dcMotor.get("Front Left");
        FrontRight = hardwareMap.dcMotor.get("Front Right");
        BackLeft = hardwareMap.dcMotor.get("Back Left");
        BackRight = hardwareMap.dcMotor.get("Back Right");

        BackRight.setDirection(DcMotor.Direction.REVERSE);
        FrontLeft.setDirection(DcMotor.Direction.REVERSE);

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

        telemetry.addData("Robot is ready to go", null);
        telemetry.update();

        waitForStart();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        if (opModeIsActive()) {
            turn(90);
            telemetry.addData("has done turn", null);
            telemetry.update();
        }

    }

    //Turn Methods

    void turn(double target) {
        angles = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double origin = angles.firstAngle;
        telemetry.addData("Can read from firstAngle: ", true);
        telemetry.update();
        sleep(1000);
        boolean turnRight = directionCheck(target, origin);
        telemetry.addData("turnRight: ", true);
        telemetry.update();
        sleep(1000);
        double distance = calculateTurnDistance(target, origin);
        telemetry.addData("calculate turn distance: ", true);
        telemetry.update();
        sleep(1000);
        double halfway = calculateHalfway(target, angles.firstAngle, distance);
        telemetry.addData("calculate turn halfway: ", true);
        telemetry.update();
        sleep(1000);
        beginAcceleration(proportionalCoefficient, distance, turnRight);
        telemetry.addData("begin moving: ", true);
        telemetry.update();
        sleep(1000);
        while (FrontLeft.isBusy() || FrontRight.isBusy() || BackLeft.isBusy() || BackRight.isBusy()) {
            if (halfwayCheck(halfway, turnRight)) {
                beginDecceleration(proportionalCoefficient, distance, turnRight);
                telemetry.addData("slow down: ", true);
                telemetry.update();
                sleep(1000);
            }
            if (angles.firstAngle == target) {
                telemetry.addData("has reached target: ", true);
                telemetry.update();
                sleep(1000);
                FrontRight.setPower(0);
                FrontLeft.setPower(0);
                BackLeft.setPower(0);
                BackRight.setPower(0);
            }
        }

    }

    //Calculation Methods

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
        if (origin > target && target >= 0) {
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



    //Telemetry Methods

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
