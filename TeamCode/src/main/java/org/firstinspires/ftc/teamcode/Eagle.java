package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import static java.lang.Thread.sleep;

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
    private Servo servoBlue;
    private CRServo servoTeamMarker;

    private HardwareMap hwMap;

    private static final int MOTOR_TICK_COUNTS = 1120;
    private static final double ARM_MAX_RANGE = 0.73d;
    private static final double ARM_MIN_RANGE = 0.0d;
    private static final double ARM_HOME = 0.0d;
    private static final double ARM_SPEED = 0.0055;
    private double servoPosition = ARM_HOME;

    //TensorFlow
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private static final String VUFORIA_KEY =
            "Aaaehpz/////AAABmQe94sn1BEXXs4h6LleDQVpPr0LBiNWsZOi8Ttm5s7saUDJ2rtO+exhVvNUTvMRXFdiHA6yjUXvRi9YUnj+8xLWXnGUJsLWdJAew7b63OOpzhlcbhzYfAujCjx4+K3GpIV2aqH3ROTQxzorjqti8Q47zhuW75aMwYYHPeqjBMpp4RO+R7z/OXuy0QmQmT1xCsOdGVUC6T5OSOChfK7OjhDL7+Ud707Uwqc/8WcLEX1PQaRsnf2nI49jENHNPfFqLg7oSMZ6fGUiIQbWKFbZEbKBjI13gkIU0VSSKi2WrspIgtg6Nm4Tau5qtzA0LhAXwS0ucFuB1PSP9VZGudONxyHGy8e/Yqy1YtmHjEvDITJWg";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;


    //Constructor
    public Eagle() {
        //Empty Constructor
    }


    public void initHardware(HardwareMap ahWMap) {
        //Initialize

        hwMap = ahWMap;

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
        servoBlue = hwMap.get(Servo.class, "servoBlue");
        servoTeamMarker = hwMap.get(CRServo.class, "servoTeamMarker");

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
        servoBlue.setDirection(Servo.Direction.REVERSE);
        servoTeamMarker.setDirection(CRServo.Direction.FORWARD);

        //Set Mode
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        servoLeft.setPosition(ARM_HOME);
        servoRight.setPosition(ARM_HOME);
        servoClaw.setPosition(ARM_HOME);
        servoBlue.setPosition(ARM_HOME);

    }

    public void initTF(HardwareMap ahwMap) {
        initVuforia(ahwMap);
        initTfod(ahwMap);
        tfod.activate();
    }

    public void TF_shutdown() {
        tfod.shutdown();
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
            motorLift.setPower(1);
        } else if(power2) {
            motorLift.setPower(-0.6);
        } else {
            motorLift.setPower(0.0);
        }
    }

    public void intake(boolean power1, boolean power2) {
        if(power1) {
            intakeRight.setPower(1);
            intakeLeft.setPower(1);
        } else if(power2) {
            intakeRight.setPower(-1);
            intakeLeft.setPower(-1);
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
            servoClaw.setPosition(0.45);
        } else if(power2) {
            servoClaw.setPosition(0.0);
        } else {
            servoClaw.setPosition(servoClaw.getPosition());
        }
    }

    public void actionServoBlue(boolean power1, boolean power2) {
        if(power1) {
            servoBlue.setPosition(0.45);
        } else if(power2) {
            servoBlue.setPosition(0.0);
        } else {
            servoBlue.setPosition(servoBlue.getPosition());
        }
    }

    public void actionServoTeamMarker(boolean power1, boolean power2) {
        if(power1) {
            servoTeamMarker.setPower(0.4);
        } else if(power2) {
            servoTeamMarker.setPower(-0.4);
        } else {
            servoTeamMarker.setPower(0);
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

    public void strafeBackward(double distance) {
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

        leftFrontDrive.setPower(-0.35);
        leftBackDrive.setPower(0.35);
        rightFrontDrive.setPower(-0.35);
        rightBackDrive.setPower(0.35);

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

    public void moveLeft(double distance) {
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
        rightFrontDrive.setTargetPosition(-target);
        rightBackDrive.setTargetPosition(-target);

        leftFrontDrive.setPower(0.35);
        leftBackDrive.setPower(0.35);
        rightFrontDrive.setPower(0.35);
        rightBackDrive.setPower(0.35);

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

    public void moveRight(double distance) {
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

        leftFrontDrive.setPower(-0.35);
        leftBackDrive.setPower(-0.35);
        rightFrontDrive.setPower(0.35);
        rightBackDrive.setPower(0.35);

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

    public void navigateRight(double distance) {
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

        leftFrontDrive.setPower(-0.65);
        leftBackDrive.setPower(-0.65);
        rightFrontDrive.setPower(0.65);
        rightBackDrive.setPower(0.65);

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

    public void navigateLeft(double distance) {
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
        rightFrontDrive.setTargetPosition(-target);
        rightBackDrive.setTargetPosition(-target);

        leftFrontDrive.setPower(0.65);
        leftBackDrive.setPower(0.65);
        rightFrontDrive.setPower(0.65);
        rightBackDrive.setPower(0.65);

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

    public void takeSkyStone() {
        servoBlue.setPosition(0.48);
    }

    public void makeSampleBlue() throws InterruptedException {

        boolean found = false;

        String position = null;

        for(int i = 1; i <= 50 && !found; i ++) {

            sleep(10);

            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

                if (updatedRecognitions != null) {

                    if (updatedRecognitions.size() == 2) {
                        int stoneX = -1;
                        int skyStoneX = -1;

                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                                skyStoneX = (int) recognition.getLeft();
                            }  else {
                                stoneX = (int) recognition.getLeft();
                            }
                        }

                        if (skyStoneX != -1 && stoneX != -1) {
                            if (skyStoneX < stoneX) {
                                //Position center
                                moveRight(7);
                                strafeForward(40);
                                takeSkyStone();
                                found = true;
                                position = "center";
                            } else {
                                //Position right
                                moveRight(30);
                                strafeForward(40);
                                takeSkyStone();
                                found = true;
                                position = "right";
                            }
                        }
                    }
                }
            }
        }

        if(!found) {
            //Position Left
            moveLeft(12);
            strafeForward(40);
            takeSkyStone();
            position = "left";
        }

        sleep(500);

        switch (position) {
            case "left" :
                strafeBackward(35);
                navigateRight(180);
                //Ma intorc dupa celalalt
            break;

            case "right" :
                strafeBackward(35);
                navigateRight(80);
                //Ma intorc dupa celalalt
            break;

            case "center" :
                strafeBackward(35);
                navigateRight(140);
                //Ma intorc dupa celalalt
            break;
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
        tfodParameters.minimumConfidence = 0.75;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

}
