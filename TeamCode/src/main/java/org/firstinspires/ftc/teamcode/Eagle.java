package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaSkyStone;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

//change
public class Eagle {

    //Chasis
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;

    //Intake
    private DcMotor intakeLeft;
    private DcMotor intakeRight;

    //Lift
    private DcMotor motorLift;

    //Servos
    private Servo servoLeft;
    private Servo servoRight;
    private Servo servoClaw;
    private Servo servoLateral;

    private HardwareMap hwMap;

    private static final int MOTOR_TICK_COUNTS = 1120;
    private static final double ARM_MAX_RANGE = 0.7d;
    private static final double ARM_MIN_RANGE = 0.0d;
    private static final double ARM_HOME = 0.0d;
    private static final double ARM_SPEED = 0.0035;
    private double servoPosition = ARM_HOME;


    //Autonomous constants

    private static final double TargetHeightRatio = 0.8;

    //TensorFlow

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private static final String VUFORIA_KEY =
            "Aaaehpz/////AAABmQe94sn1BEXXs4h6LleDQVpPr0LBiNWsZOi8Ttm5s7saUDJ2rtO+exhVvNUTvMRXFdiHA6yjUXvRi9YUnj+8xLWXnGUJsLWdJAew7b63OOpzhlcbhzYfAujCjx4+K3GpIV2aqH3ROTQxzorjqti8Q47zhuW75aMwYYHPeqjBMpp4RO+R7z/OXuy0QmQmT1xCsOdGVUC6T5OSOChfK7OjhDL7+Ud707Uwqc/8WcLEX1PQaRsnf2nI49jENHNPfFqLg7oSMZ6fGUiIQbWKFbZEbKBjI13gkIU0VSSKi2WrspIgtg6Nm4Tau5qtzA0LhAXwS0ucFuB1PSP9VZGudONxyHGy8e/Yqy1YtmHjEvDITJWg";

    private VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;


    //Constructor
    public Eagle() {
        //Empty Constructor
    }


    public void initHardware(HardwareMap hardwareMap) {
        //Initialize

        hwMap = hardwareMap;

        leftFrontDrive  = hwMap.get(DcMotor.class, "leftFrontDrive");
        leftBackDrive   = hwMap.get(DcMotor.class, "leftBackDrive");
        rightFrontDrive = hwMap.get(DcMotor.class, "rightFrontDrive");
        rightBackDrive  = hwMap.get(DcMotor.class, "rightBackDrive");

        intakeLeft      = hwMap.get(DcMotor.class, "intakeLeft");
        intakeRight     = hwMap.get(DcMotor.class, "intakeRight");

        motorLift       = hwMap.get(DcMotor.class, "motorLift");

        //Servo
        servoLeft = hwMap.get(Servo.class, "servoLeft");
        servoRight = hwMap.get(Servo.class, "servoRight");
        servoClaw = hwMap.get(Servo.class, "servoClaw");
        servoLateral = hwMap.get(Servo.class, "servoLateral");

        //Set Direction
        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeRight.setDirection(DcMotorSimple.Direction.FORWARD);

        motorLift.setDirection(DcMotorSimple.Direction.FORWARD);

        servoLeft.setDirection(Servo.Direction.REVERSE);
        servoRight.setDirection(Servo.Direction.FORWARD);
        servoClaw.setDirection(Servo.Direction.FORWARD);
        servoLateral.setDirection(Servo.Direction.REVERSE);

        //Set Mode
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        servoLeft.setPosition(ARM_HOME);
        servoRight.setPosition(ARM_HOME);
        servoClaw.setPosition(ARM_HOME);
        servoLateral.setPosition(ARM_HOME);

    }

    public void initTF(HardwareMap ahwMap) {
        initVuforia(ahwMap);
        initTfod(ahwMap);
        tfod.activate();
    }

    public void manualMove(double Strafe, double Forward, double Turn) {
        //Find the magnitude of the controller's input
        double r = Math.hypot(Strafe, Forward);

        //returns point from +X axis to point (forward, strafe)
        double robotAngle = Math.atan2(Forward, Strafe) - Math.PI / 4;

        //Quantity to turn by (turn)
        double rightX = Turn;

        //double vX represents the velocities sent to each motor
        final double v1 = (r * Math.cos(robotAngle)) + rightX;
        final double v2 = (r * Math.sin(robotAngle)) - rightX;
        final double v3 = (r * Math.sin(robotAngle)) + rightX;
        final double v4 = (r * Math.cos(robotAngle)) - rightX;

        leftFrontDrive.setPower(v1);
        rightFrontDrive.setPower(v2);
        leftBackDrive.setPower(v3);
        rightBackDrive.setPower(v4);

    }

    public void moveLift(boolean power1, boolean power2) {
        if(power1) {
            motorLift.setPower(0.9);
        } else if(power2) {
            motorLift.setPower(-0.6);
        } else {
            motorLift.setPower(0.0);
        }
    }

    public void intake(boolean power1, boolean power2) {
        if(power1) {
            intakeRight.setPower(0.9);
            intakeLeft.setPower(0.9);
        } else if(power2) {
            intakeRight.setPower(-0.9);
            intakeLeft.setPower(-0.8);
        } else {
            intakeRight.setPower(0);
            intakeLeft.setPower(0);
        }
    }

    public void moveArm(boolean power1, boolean power2) {
        if(power1) {
            servoPosition += ARM_SPEED;
        } else if(power2) {
            servoPosition -= ARM_SPEED;
        } else {
            servoPosition += 0;
        }

        servoPosition = Range.clip(servoPosition, ARM_MIN_RANGE, ARM_MAX_RANGE);

        servoLeft.setPosition(servoPosition);
        servoRight.setPosition(servoPosition);
    }

    public void actionServoClaw(boolean power1, boolean power2) {
        if(power1) {
            servoClaw.setPosition(0.4);
        } else if(power2) {
            servoClaw.setPosition(0.0);
        } else {
            servoClaw.setPosition(servoClaw.getPosition());
        }
    }

    public void actionServoLateral(boolean power1, boolean power2) {
        if(power1) {
            servoLateral.setPosition(0.45);
        } else if(power2) {
            servoLateral.setPosition(0.0);
        } else {
            servoLateral.setPosition(servoLateral.getPosition());
        }
    }

    //Functii autonom

    public void strafeForward(double distance) {
        //Reset encoders
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double circumference = Math.PI * 10;
        double rotationsNeeded = distance/circumference;
        int target = (int)(MOTOR_TICK_COUNTS * rotationsNeeded);

        leftFrontDrive.setTargetPosition(target);
        leftBackDrive.setTargetPosition(target);
        rightFrontDrive.setTargetPosition(target);
        rightBackDrive.setTargetPosition(target);

        leftFrontDrive.setPower(0.35);
        leftBackDrive.setPower(-0.35);
        rightFrontDrive.setPower(0.35);
        rightBackDrive.setPower(-0.35);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while(leftFrontDrive.isBusy() && rightFrontDrive.isBusy() && leftBackDrive.isBusy() && rightBackDrive.isBusy()) {
            //wait
        }

        leftFrontDrive.setPower(0.0);
        leftBackDrive.setPower(0.0);
        rightFrontDrive.setPower(0.0);
        rightBackDrive.setPower(0.0);


    }

    public void moveLeft() {

    }

    public void moveRight() {

    }

    public void takeSkyStone() {
        servoLateral.setPosition(0.45);
    }

    public void searchSkystone() {
        // We'll loop until gold block captured or time is up
        boolean SkystoneFound = false;
        while (!SkystoneFound) {
            // Get list of current recognitions.
            List<Recognition> recognitions = tfod.getRecognitions();
            // Report number of recognitions.
            //telemetry.addData("Objects Recognized", recognitions.size());
            // If some objects detected...
            if (recognitions.size() > 0) {
                // ...let's count how many are gold.
                int SkystoneCount = 0;
                // Step through the stones detected.
                for (Recognition recognition : recognitions) {
                    if (recognition.getLabel().equals("Skystone")) {
                        // A Skystone has been detected.
                        SkystoneCount ++;
                        // We can assume this is the first Skystone
                        // because we break out of this loop below after
                        // using the information from the first Skystone.
                        // We don't need to calculate turn angle to Skystone
                        // because TensorFlow has estimated it for us.
                        double ObjectAngle = recognition.estimateAngleToObject(AngleUnit.DEGREES);
                        // Negative angle means Skystone is left, else right.
                        //telemetry.addData("Estimated Angle", ObjectAngle);
                        if (ObjectAngle > 0) {
                            //telemetry.addData("Direction", "Right");
                        } else {
                            //telemetry.addData("Direction", "Left");
                        }
                        // Calculate power levels for turn toward Skystone.
                        double LeftPower = -0.25 * (ObjectAngle / 45);
                        double RightPower = -0.25 * (ObjectAngle / 45);
                        // We'll be comparing the Skystone height
                        // to the height of the video image to estimate
                        // how close the robot is to the Skystone.
                        double ImageHeight = recognition.getImageHeight();
                        double ObjectHeight = recognition.getHeight();
                        // Calculate height of Skystone relative to image height.
                        // Larger ratio means robot is closer to Skystone.
                        double ObjectHeightRatio = ObjectHeight / ImageHeight;
                        //telemetry.addData("HeightRatio", ObjectHeightRatio);
                        // Use height ratio to determine distance.
                        // If height ratio larger than (target - tolerance)...
                        if (ObjectHeightRatio < TargetHeightRatio - 0.05) {
                            // ...not close enough yet.
                            //telemetry.addData("Distance", "Not close enough");
                            // If sum of turn powers are small
                            if (Math.abs(LeftPower) + Math.abs(RightPower) < 0.2) {
                                // ...don't really need to turn.  Move forward.
                                //telemetry.addData("Action", "Forward");
                                // Go forward by setting power proportional to how
                                // far from target distance.
                                LeftPower = 0.035 + 0.5 * ((TargetHeightRatio - 0.05) - ObjectHeightRatio);
                                RightPower = LeftPower;
                            } else {
                                // Else we'll turn to Skystone with current power levels.
                                //telemetry.addData("Action", "Turn");
                            }
                            // Else if height ratio more than (target+tolerance)...
                        } else if (ObjectHeightRatio > TargetHeightRatio + 0.05) {
                            // ...robot too close to Skystone.
                            //telemetry.addData("Distance", "Too close");
                            // If calculated turn power levels are small...
                            if (Math.abs(LeftPower) + Math.abs(RightPower) < 0.12) {
                                // ...don't need to turn.  Backup instead by setting
                                // power proportional to how far past target ratio
                                //telemetry.addData("Action", "Back up");
                                LeftPower = -0.05 + -0.5 * ((TargetHeightRatio + 0.05) - TargetHeightRatio);
                                RightPower = LeftPower;
                            } else {
                                // Else use current power levels to turn to Skystone
                                //telemetry.addData("Action", "Turn");
                            }
                        } else {
                            // Skystone is about one neck length away.
                            //telemetry.addData("Distance", "Correct");
                            // If calculated turn power levels are small...
                            if (Math.abs(LeftPower) + Math.abs(RightPower) < 0.12) {
                                // ...robot is centered on the Skystone.
                                //telemetry.addData("Action", "Motors off, hit the Skystone");
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
                                //telemetry.addData("Action", "Turn");
                            }
                        }
                        //telemetry.addData("Left Power", LeftPower);
                        //telemetry.addData("Right Power", RightPower);
                        // Set power levels to get closer to Skystone.
                        leftFrontDrive.setPower(LeftPower);
                        leftBackDrive.setPower(LeftPower);
                        rightFrontDrive.setPower(RightPower);
                        rightBackDrive.setPower(RightPower);
                        // We've found a Skystone so we don't have
                        // to look at rest of detected objects.
                        // Break out of For-each-recognition.
                        break;
                    }
                }
                // If no Skystones detected...
                if (SkystoneCount == 0) {
                    //telemetry.addData("Status", "No Skystone");
                    //telemetry.addData("Action", "Back up");
                    // Back up slowly hoping to bring Skystone in view.
//                    LeftMotor.setPower(-0.1);
//                    RightMotor.setPower(-0.1);
                }
            } else {
                // No objects detected
                //telemetry.addData("Status", "No objects detected");
                //telemetry.addData("Action", "Back up");
                // Back up slowly hoping to bring objects in view.
//                LeftMotor.setPower(-0.1);
//                RightMotor.setPower(-0.1);
            }
            //telemetry.update();
        }
    }




    private void initVuforia(HardwareMap bhwMap) {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = bhwMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod(HardwareMap chwMap) {
        int tfodMonitorViewId = chwMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", chwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.7;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }



}
