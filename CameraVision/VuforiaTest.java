package org.firstinspires.ftc.teamcode.CameraVision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import java.util.ArrayList;

@Autonomous
@Disabled
public class VuforiaTest extends LinearOpMode{

    //VUFORIA STUFF
    public static final String TAG = "Vuforia Navigation Sample";

    OpenGLMatrix lastLocation = null;

    VuforiaLocalizer vuforia;

    //TFOD STUFF
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private TFObjectDetector tfod;

    @Override
    public void runOpMode(){

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);


        parameters.vuforiaLicenseKey = "AR/sArn/////AAABmYT4OHM7O0ghuzdm7dDNDFhZG6Yl6O3GSKrhCfuGTcO9wTvGG646aPxMUSyLSgqkS7u8oRDM8K744VEXHCOYpsfzxT7Gp/Evu8537krH3m91cimF8TITI5nvIsA9bOXSqInL1u+X0uGgU9ZA44A0Ox4nJy/2Yi7YhlPrPl9A30bgrkY+/azNr1SGVGucWZQHACG9/6NkO5KhyWL5ImeiNEsE10mIqvA9FkcC8iWA5ANwGmtsGkzgW01HgYy43BIP2zULL9Bq44vzCda7agK03HvYqgAasu3D8KSI3ecm5E+hvJiFIe62ptFx728jErgtOr5gyKV5oo8nO60x74XUXuu1kpptLjwxXHQOk0h4CHFc";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        //TARGET SETUP
        VuforiaTrackables RoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");

        VuforiaTrackable BluePerimeter = RoverRuckus.get(0);
        BluePerimeter.setName("BluePerimeter");

        VuforiaTrackable RedPerimeter = RoverRuckus.get(1);
        RedPerimeter.setName("RedPerimeter");

        VuforiaTrackable FrontPerimeter = RoverRuckus.get(2);
        FrontPerimeter.setName("FrontPerimeter");

        VuforiaTrackable BackPerimeter = RoverRuckus.get(3);
        BackPerimeter.setName("BackPerimeter");

        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(RoverRuckus);

        //TARGET LOCATIONS ON FIELD
        float mmPerInch        = 25.4f;
        float mmBotWidth       = 18 * mmPerInch;
        float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;

        OpenGLMatrix BluePerimeterTargetOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth/2, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ,
                        AngleUnit.DEGREES, 90, 0, 0));
        BluePerimeter.setLocation(BluePerimeterTargetOnField);

        OpenGLMatrix RedPerimeterTargetOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth/2, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ,
                        AngleUnit.DEGREES, 90, 0, 180));
        RedPerimeter.setLocation(RedPerimeterTargetOnField);

        OpenGLMatrix FrontPerimeterTargetOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth/2, 0, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ,
                        AngleUnit.DEGREES, 90, 0, -90));
        FrontPerimeter.setLocation(FrontPerimeterTargetOnField);

        OpenGLMatrix BackPerimeterTargetOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth/2, 0, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ,
                        AngleUnit.DEGREES, 90, 0, 90));
        BackPerimeter.setLocation(BackPerimeterTargetOnField);

        //PHONE LOCATION ON ROBOT
        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(mmBotWidth/2,mmBotWidth/2,0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ,
                        AngleUnit.DEGREES, 90, 0, 0));

        //INSTANTIATE LISTENERS
        ((VuforiaTrackableDefaultListener)BluePerimeter.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)RedPerimeter.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)FrontPerimeter.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)BackPerimeter.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);

        //ROBOT READY FOR START
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        //INSTANTIATE TRACKABLES
        RoverRuckus.activate();

        //INIT TFOD
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);

        tfod.activate();

        //START TRACKING
        while (opModeIsActive()) {

            //VUFORIA LOOP
            for (VuforiaTrackable trackable : allTrackables) {
                //DETERMINES IF ROBOT CAN SEE TARGET OR NOT
                telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //
                //UPDATES lastLocation VARIABLE
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();

                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }

            }

            //TFOD LOOP
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() == 3) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getLeft();
                            } else {
                                silverMineral2X = (int) recognition.getLeft();
                            }
                        }
                        if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                            if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                telemetry.addData("Gold Mineral Position", "Left");
                            } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                telemetry.addData("Gold Mineral Position", "Right");
                            } else {
                                telemetry.addData("Gold Mineral Position", "Center");
                            }
                        }
                    }
                    telemetry.update();
                }
            }


            //PRINTS ROBOT LOCATION ON PHONE
            if (lastLocation != null) {
                telemetry.addData("Pos", format(lastLocation));
            } else {
                telemetry.addData("Pos", "Unknown");
            }
            telemetry.update();
        }
    }

    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }

}
