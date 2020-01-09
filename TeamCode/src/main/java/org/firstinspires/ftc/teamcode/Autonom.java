package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaSkyStone;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.TfodSkyStone;

import java.util.List;

@Autonomous(name = "Autonom", group = "Autonomous")

public class Autonom extends LinearOpMode {

    private Eagle eagle = new Eagle();

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private static final String VUFORIA_KEY =
            "Aaaehpz/////AAABmQe94sn1BEXXs4h6LleDQVpPr0LBiNWsZOi8Ttm5s7saUDJ2rtO+exhVvNUTvMRXFdiHA6yjUXvRi9YUnj+8xLWXnGUJsLWdJAew7b63OOpzhlcbhzYfAujCjx4+K3GpIV2aqH3ROTQxzorjqti8Q47zhuW75aMwYYHPeqjBMpp4RO+R7z/OXuy0QmQmT1xCsOdGVUC6T5OSOChfK7OjhDL7+Ud707Uwqc/8WcLEX1PQaRsnf2nI49jENHNPfFqLg7oSMZ6fGUiIQbWKFbZEbKBjI13gkIU0VSSKi2WrspIgtg6Nm4Tau5qtzA0LhAXwS0ucFuB1PSP9VZGudONxyHGy8e/Yqy1YtmHjEvDITJWg";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;


    @Override
    public void runOpMode() {

        // Initialization
        telemetry.addData("Init ", "started");
        telemetry.update();

        initVuforia();
        initTfod();

        telemetry.addData("Vuforia & Tfod", "initialized");
        telemetry.update();

        eagle.init(hardwareMap);

        telemetry.addData("Robot", "initialized");
        telemetry.update();

        tfod.activate();

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        // Set target ratio of object height to image
        // height value corresponding to the length
        // of the robot's neck.
        double TargetHeightRatio = 0.8;

        waitForStart();

        // We'll loop until gold block captured or time is up
        boolean SkystoneFound = false;
        while (opModeIsActive() && !SkystoneFound) {
            // Get list of current recognitions.
            List<Recognition> recognitions = tfod.getRecognitions();
            // Report number of recognitions.
            telemetry.addData("Objects Recognized", recognitions.size());
            // If some objects detected...
            if (recognitions.size() > 0) {
                // ...let's count how many are gold.
                int SkystoneCount = 0;
                // Step through the stones detected.
                for (Recognition recognition : recognitions) {
                    if (recognition.getLabel().equals("Skystone")) {
                        // A Skystone has been detected.
                        SkystoneCount = SkystoneCount + 1;
                        // We can assume this is the first Skystone
                        // because we break out of this loop below after
                        // using the information from the first Skystone.
                        // We don't need to calculate turn angle to Skystone
                        // because TensorFlow has estimated it for us.
                        double ObjectAngle = recognition.estimateAngleToObject(AngleUnit.DEGREES);
                        // Negative angle means Skystone is left, else right.
                        telemetry.addData("Estimated Angle", ObjectAngle);
                        if (ObjectAngle > 0) {
                            telemetry.addData("Direction", "Right");
                        } else {
                            telemetry.addData("Direction", "Left");
                        }
                        // Calculate power levels for turn toward Skystone.
                        double LeftPower = 0.25 * (ObjectAngle / 45);
                        double RightPower = -0.25 * (ObjectAngle / 45);
                        // We'll be comparing the Skystone height
                        // to the height of the video image to estimate
                        // how close the robot is to the Skystone.
                        double ImageHeight = recognition.getImageHeight();
                        double ObjectHeight = recognition.getHeight();
                        // Calculate height of Skystone relative to image height.
                        // Larger ratio means robot is closer to Skystone.
                        double ObjectHeightRatio = ObjectHeight / ImageHeight;
                        telemetry.addData("HeightRatio", ObjectHeightRatio);
                        // Use height ratio to determine distance.
                        // If height ratio larger than (target - tolerance)...
                        if (ObjectHeightRatio < TargetHeightRatio - 0.05) {
                            // ...not close enough yet.
                            telemetry.addData("Distance", "Not close enough");
                            // If sum of turn powers are small
                            if (Math.abs(LeftPower) + Math.abs(RightPower) < 0.2) {
                                // ...don't really need to turn.  Move forward.
                                telemetry.addData("Action", "Forward");
                                // Go forward by setting power proportional to how
                                // far from target distance.
                                LeftPower = 0.035 + 0.5 * ((TargetHeightRatio - 0.05) - ObjectHeightRatio);
                                RightPower = LeftPower;
                            } else {
                                // Else we'll turn to Skystone with current power levels.
                                telemetry.addData("Action", "Turn");
                            }
                            // Else if height ratio more than (target+tolerance)...
                        } else if (ObjectHeightRatio > TargetHeightRatio + 0.05) {
                            // ...robot too close to Skystone.
                            telemetry.addData("Distance", "Too close");
                            // If calculated turn power levels are small...
                            if (Math.abs(LeftPower) + Math.abs(RightPower) < 0.12) {
                                // ...don't need to turn.  Backup instead by setting
                                // power proportional to how far past target ratio
                                telemetry.addData("Action", "Back up");
                                LeftPower = -0.05 + -0.5 * ((TargetHeightRatio + 0.05) - TargetHeightRatio);
                                RightPower = LeftPower;
                            } else {
                                // Else use current power levels to turn to Skystone
                                telemetry.addData("Action", "Turn");
                            }
                        } else {
                            // Skystone is about one neck length away.
                            telemetry.addData("Distance", "Correct");
                            // If calculated turn power levels are small...
                            if (Math.abs(LeftPower) + Math.abs(RightPower) < 0.12) {
                                // ...robot is centered on the Skystone.
                                telemetry.addData("Action", "Motors off, hit the Skystone");
                                // Turn motors off by setting power to 0.
                                LeftPower = 0;
                                RightPower = 0;
                                // Lower neck and open jaw.
//                                LowerServo.setPosition(0.5);
//                                UpperServo.setPosition(0.5);
                                SkystoneFound = true;
                            } else {
                                // Otherwise use current power levels to turn
                                // to better center on gold.
                                telemetry.addData("Action", "Turn");
                            }
                        }
                        telemetry.addData("Left Power", LeftPower);
                        telemetry.addData("Right Power", RightPower);
                        // Set power levels to get closer to Skystone.
//                        LeftMotor.setPower(LeftPower);
//                        RightMotor.setPower(RightPower);
                        // We've found a Skystone so we don't have
                        // to look at rest of detected objects.
                        // Break out of For-each-recognition.
                        break;
                    }
                }
                // If no Skystones detected...
                if (SkystoneCount == 0) {
                    telemetry.addData("Status", "No Skystone");
                    telemetry.addData("Action", "Back up");
                    // Back up slowly hoping to bring Skystone in view.
//                    LeftMotor.setPower(-0.1);
//                    RightMotor.setPower(-0.1);
                }
            } else {
                // No objects detected
                telemetry.addData("Status", "No objects detected");
                telemetry.addData("Action", "Back up");
                // Back up slowly hoping to bring objects in view.
//                LeftMotor.setPower(-0.1);
//                RightMotor.setPower(-0.1);
            }
            telemetry.update();
        }

        // Skystone found, time is up or stop was requested.
        tfod.shutdown();
        // Pause to let driver station to see last telemetry.
        sleep(2000);

    }

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

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.7;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

}