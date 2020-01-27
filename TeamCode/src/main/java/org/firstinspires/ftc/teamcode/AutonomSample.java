package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@TeleOp(name = "Concept: TensorFlow Object Detection Webcam", group = "Concept")

public class AutonomSample extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";


    private static final String VUFORIA_KEY =
            "Aaaehpz/////AAABmQe94sn1BEXXs4h6LleDQVpPr0LBiNWsZOi8Ttm5s7saUDJ2rtO+exhVvNUTvMRXFdiHA6yjUXvRi9YUnj+8xLWXnGUJsLWdJAew7b63OOpzhlcbhzYfAujCjx4+K3GpIV2aqH3ROTQxzorjqti8Q47zhuW75aMwYYHPeqjBMpp4RO+R7z/OXuy0QmQmT1xCsOdGVUC6T5OSOChfK7OjhDL7+Ud707Uwqc/8WcLEX1PQaRsnf2nI49jENHNPfFqLg7oSMZ6fGUiIQbWKFbZEbKBjI13gkIU0VSSKi2WrspIgtg6Nm4Tau5qtzA0LhAXwS0ucFuB1PSP9VZGudONxyHGy8e/Yqy1YtmHjEvDITJWg";


    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private Eagle eagle = new Eagle();

    @Override
    public void runOpMode() {

        initVuforia();

        initTfod();

        eagle.initHardware(hardwareMap);

        tfod.activate();

        boolean isFound = false;

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isFound) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

                if (updatedRecognitions != null) {

                  telemetry.addData("# Object Detected", updatedRecognitions.size());

                  if(updatedRecognitions.size() == 3) {
                      int stone1X = -1;
                      int stone2X = -1;
                      int skyStoneX = -1;

                      for (Recognition recognition : updatedRecognitions) {
                          if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                              skyStoneX = (int) recognition.getLeft();
                          } else if (stone1X == -1) {
                              stone1X = (int) recognition.getLeft();
                          } else {
                              stone2X = (int) recognition.getLeft();
                          }
                      }

                      if (skyStoneX != -1 && stone1X != -1 && stone2X != -1) {
                          if (skyStoneX < stone1X && skyStoneX < stone2X) {
                              telemetry.addData("Skystone position", "left");
                              eagle.moveLeft();
                              eagle.strafeForward(20);
                              isFound = true;
                          } else if (skyStoneX > stone1X && skyStoneX > stone2X) {
                              telemetry.addData("Skystone position", "right");
                              eagle.moveRight();
                              eagle.strafeForward(20);
                              isFound = true;
                          } else {
                              telemetry.addData("SkyStone position", "center");
                              eagle.strafeForward(40);
                              isFound = true;
                          }
                      }
                  } else {
                      //Back up slowly
                      telemetry.addData("Action", "backing up slowly");
                  }
                  telemetry.update();
                }
            }
        }

        tfod.shutdown();

    }

    /**
     * Initialize the Vuforia localization engine.
     */
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
       tfodParameters.minimumConfidence = 0.8;
       tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
       tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
