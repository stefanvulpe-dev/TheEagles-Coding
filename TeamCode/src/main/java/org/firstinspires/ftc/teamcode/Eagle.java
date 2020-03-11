package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import static java.lang.Thread.sleep;

//change
public class Eagle {

    //Chasis
    public DcMotor leftFrontDrive;
    public DcMotor leftBackDrive;
    public DcMotor rightFrontDrive;
    public DcMotor rightBackDrive;

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
    private Servo servoRed;
    private Servo servoBlueClaw;
    private Servo servoRedClaw;

    private Servo servoPlateRight;
    private Servo servoPlateLeft;

    private CRServo servoTeamMarker;
    private CRServo servoParking;

    private DistanceSensor sensorRange;
    private DigitalChannel digitalTouch;

    private HardwareMap hwMap;

    private static final int MOTOR_TICK_COUNTS = 1120;
    private static final double wheelsRatio = 1.5d;

    private static final double ARM_MAX_RANGE = 0.8d;
    private static final double ARM_MIN_RANGE = 0.0d;
    private static final double ARM_HOME = 0.0d;
    private static final double ARM_SPEED = 0.08;

    private static final double liftSpeed = 0.25d;
    private static final double LIFT_MAX_SPEED = 1.0d;
    private static final double LIFT_MIN_SPEED = -1.0d;

    private double liftPower = 0.0d;
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
        servoRed = hwMap.get(Servo.class, "servoRed");

        servoBlueClaw = hwMap.get(Servo.class, "servoBlueClaw");
        servoRedClaw = hwMap.get(Servo.class, "servoRedClaw");

        servoPlateRight = hwMap.get(Servo.class, "servoPlateRight");
        servoPlateLeft = hwMap.get(Servo.class, "servoPlateLeft");

        servoTeamMarker = hwMap.get(CRServo.class, "servoTeamMarker");
        servoParking = hwMap.get(CRServo.class, "servoParking");

        //Senzori
        sensorRange = hwMap.get(DistanceSensor.class, "sensor_range");
        digitalTouch = hwMap.get(DigitalChannel.class, "sensor_digital");

        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

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

        servoBlue.setDirection(Servo.Direction.FORWARD);
        servoRed.setDirection(Servo.Direction.REVERSE);

        servoBlueClaw.setDirection(Servo.Direction.FORWARD);
        servoRedClaw.setDirection(Servo.Direction.REVERSE);

        servoPlateRight.setDirection(Servo.Direction.REVERSE);
        servoPlateLeft.setDirection(Servo.Direction.FORWARD);

        servoTeamMarker.setDirection(CRServo.Direction.FORWARD);
        servoParking.setDirection(DcMotorSimple.Direction.FORWARD);

        //Set Mode
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servoLeft.setPosition(ARM_HOME);
        servoRight.setPosition(ARM_HOME);
        servoClaw.setPosition(ARM_HOME);
        servoBlue.setPosition(ARM_HOME);
        servoRed.setPosition(ARM_HOME);
        servoBlueClaw.setPosition(ARM_HOME);
        servoRedClaw.setPosition(ARM_HOME);
        servoPlateRight.setPosition(ARM_HOME);
        servoPlateLeft.setPosition(ARM_HOME);

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
        double v1 = Range.clip(r * Math.cos(robotAngle) + rightX, -1.0, 1.0);
        double v2 = Range.clip(r * Math.sin(robotAngle) - rightX, -1.0, 1.0);
        double v3 = Range.clip(r * Math.sin(robotAngle) + rightX, -1.0, 1.0);
        double v4 = Range.clip(r * Math.cos(robotAngle) - rightX, -1.0, 1.0);

        leftFrontDrive.setPower(v1);
        rightFrontDrive.setPower(v2);
        leftBackDrive.setPower(v3);
        rightBackDrive.setPower(v4);

    }

    public void constructionMode(double Strafe, double Forward, double Turn) {
        double r = Math.hypot(Strafe, Forward);

        //returns point from +X axis to point (forward, strafe)
        double robotAngle = Math.atan2(Forward, Strafe) - Math.PI / 4;

        //Quantity to turn by (turn)
        double rightX = Turn;

        //double vX represents the velocities sent to each motor
        double v1 = Range.clip(r * Math.cos(robotAngle) + rightX, -0.4, 0.4);
        double v2 = Range.clip(r * Math.sin(robotAngle) - rightX, -0.4, 0.4);
        double v3 = Range.clip(r * Math.sin(robotAngle) + rightX, -0.4, 0.4);
        double v4 = Range.clip(r * Math.cos(robotAngle) - rightX, -0.4, 0.4);

        leftFrontDrive.setPower(v1);
        rightFrontDrive.setPower(v2);
        leftBackDrive.setPower(v3);
        rightBackDrive.setPower(v4);
    }

    public void moveLift(boolean power1, boolean power2) throws InterruptedException {
        if(power1) {
            liftPower += liftSpeed;
        } else if(power2) {
            liftPower -= liftSpeed;
        } else {
            liftPower = 0.0d;
        }
        //Switch apasat
        if(!digitalTouch.getState()) {
            liftPower = -0.10d;
            sleep(250);
        }
        liftPower = Range.clip(liftPower, LIFT_MIN_SPEED, LIFT_MAX_SPEED);
        motorLift.setPower(liftPower);
        sleep(50);
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

    /* Servo-uri Manual */

    public void actionServoClaw(boolean power1, boolean power2) {
        if(power1) {
            servoClaw.setPosition(0.32);
        } else if(power2) {
            servoClaw.setPosition(0.0);
        } else {
            servoClaw.setPosition(servoClaw.getPosition());
        }
    }

    public void actionServoPlate(double power1, double power2) {
        if(power1 > 0.0d) {
            servoPlateRight.setPosition(0.62);
            servoPlateLeft.setPosition(0.62);
        } else if(power2 > 0.0d) {
            servoPlateRight.setPosition(0.0);
            servoPlateLeft.setPosition(0.0);
        } else {
            servoPlateRight.setPosition(servoPlateRight.getPosition());
            servoPlateLeft.setPosition(servoPlateLeft.getPosition());
        }
    }

    public void actionServoTeamMarker(boolean power1, boolean power2) {
        if(power1) {
            servoTeamMarker.setPower(0.2);
        } else if(power2) {
            servoTeamMarker.setPower(-0.2);
        } else {
            servoTeamMarker.setPower(0);
        }
    }

    public void actionServoParking(double power1, double power2) {
        if(power1 > 0.0d) {
            servoParking.setPower(1.0);
        } else if(power2 > 0.0d) {
            servoParking.setPower(-1.0);
        } else {
            servoParking.setPower(0.0);
        }
    }

    /* Motoare autonom */

    public void strafeForward(double distance) {
        //Reset encoders
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double circumference = Math.PI * 10;
        double rotationsNeeded = (distance/circumference) / wheelsRatio;
        int target = (int)(MOTOR_TICK_COUNTS * rotationsNeeded);

        leftFrontDrive.setTargetPosition(-target);
        leftBackDrive.setTargetPosition(target);
        rightFrontDrive.setTargetPosition(-target);
        rightBackDrive.setTargetPosition(target);

        leftFrontDrive.setPower(0.5);
        leftBackDrive.setPower(0.5);
        rightFrontDrive.setPower(0.5);
        rightBackDrive.setPower(0.5);

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

    public void moveForward() {
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFrontDrive.setPower(-0.5);
        leftBackDrive.setPower(0.5);
        rightFrontDrive.setPower(0.5);
        rightBackDrive.setPower(-0.5);
    }

    private void stop() {
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    public void strafeBackward(double distance) {
        //Reset encoders
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double circumference = Math.PI * 10;
        double rotationsNeeded = (distance/circumference) / wheelsRatio;
        int target = (int)(MOTOR_TICK_COUNTS * rotationsNeeded);

        leftFrontDrive.setTargetPosition(target);
        leftBackDrive.setTargetPosition(-target);
        rightFrontDrive.setTargetPosition(target);
        rightBackDrive.setTargetPosition(-target);

        leftFrontDrive.setPower(0.5);
        leftBackDrive.setPower(0.5);
        rightFrontDrive.setPower(0.5);
        rightBackDrive.setPower(0.5);

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

        double circumference = Math.PI * 10.0d;
        double rotationsNeeded = distance/circumference;
        int target = (int)(MOTOR_TICK_COUNTS * rotationsNeeded);

        leftFrontDrive.setTargetPosition(-target);
        leftBackDrive.setTargetPosition(-target);
        rightFrontDrive.setTargetPosition(target);
        rightBackDrive.setTargetPosition(target);

        leftFrontDrive.setPower(0.5);
        leftBackDrive.setPower(0.5);
        rightFrontDrive.setPower(0.5);
        rightBackDrive.setPower(0.5);

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
        double rotationsNeeded = (distance/circumference) / wheelsRatio;
        int target = (int)(MOTOR_TICK_COUNTS * rotationsNeeded);

        leftFrontDrive.setTargetPosition(target);
        leftBackDrive.setTargetPosition(target);
        rightFrontDrive.setTargetPosition(-target);
        rightBackDrive.setTargetPosition(-target);

        leftFrontDrive.setPower(0.5);
        leftBackDrive.setPower(0.5);
        rightFrontDrive.setPower(0.5);
        rightBackDrive.setPower(0.5);

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
        double rotationsNeeded = (distance/circumference) / wheelsRatio;
        int target = (int)(MOTOR_TICK_COUNTS * rotationsNeeded);

        leftFrontDrive.setTargetPosition(target);
        leftBackDrive.setTargetPosition(target);
        rightFrontDrive.setTargetPosition(-target);
        rightBackDrive.setTargetPosition(-target);

        leftFrontDrive.setPower(0.75);
        leftBackDrive.setPower(0.75);
        rightFrontDrive.setPower(0.75);
        rightBackDrive.setPower(0.75);

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
        double rotationsNeeded = (distance/circumference) / wheelsRatio;
        int target = (int)(MOTOR_TICK_COUNTS * rotationsNeeded);

        leftFrontDrive.setTargetPosition(-target);
        leftBackDrive.setTargetPosition(-target);
        rightFrontDrive.setTargetPosition(target);
        rightBackDrive.setTargetPosition(target);

        leftFrontDrive.setPower(0.75);
        leftBackDrive.setPower(0.75);
        rightFrontDrive.setPower(0.75);
        rightBackDrive.setPower(0.75);

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

    public void turn90Right(double distance) {
        //Reset encoders
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double circumference = Math.PI * 10;
        double rotationsNeeded = (distance/circumference) / wheelsRatio;
        int target = (int)(MOTOR_TICK_COUNTS * rotationsNeeded);

        leftFrontDrive.setTargetPosition(target);
        leftBackDrive.setTargetPosition(target);
        rightFrontDrive.setTargetPosition(-target);
        rightBackDrive.setTargetPosition(-target);

        leftFrontDrive.setPower(0.75);
        leftBackDrive.setPower(0.75);
        rightFrontDrive.setPower(-0.75);
        rightBackDrive.setPower(-0.75);

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

    public void turn90Left(double distance) {
        //Reset encoders
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double circumference = Math.PI * 10;
        double rotationsNeeded = (distance/circumference) / wheelsRatio;
        int target = (int)(MOTOR_TICK_COUNTS * rotationsNeeded);

        leftFrontDrive.setTargetPosition(-target);
        leftBackDrive.setTargetPosition(-target);
        rightFrontDrive.setTargetPosition(-target);
        rightBackDrive.setTargetPosition(-target);

        leftFrontDrive.setPower(0.75);
        leftBackDrive.setPower(0.75);
        rightFrontDrive.setPower(0.75);
        rightBackDrive.setPower(0.75);

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

    /* Servo-uri Sample */


    public void takeSkyStoneRed() throws InterruptedException {
        servoRed.setPosition(0.48);
        sleep(500);
        servoRedClaw.setPosition(0.42);
        sleep(250);
        servoRed.setPosition(ARM_HOME);

    }

    private void leaveSkyStoneRed() throws InterruptedException {
        servoRed.setPosition(0.48);
        sleep(250);
        servoRedClaw.setPosition(0.25);
        sleep(250);
        servoRed.setPosition(ARM_HOME);

    }

    private void leaveSkyStoneBlue() throws InterruptedException {
        servoBlue.setPosition(0.48);
        sleep(250);
        servoBlueClaw.setPosition(0.25);
        sleep(250);
        servoBlue.setPosition(ARM_HOME);

    }

    public void takeSkyStoneBlue() throws InterruptedException {
        servoBlue.setPosition(0.48);
        sleep(500);
        servoBlueClaw.setPosition(0.42);
        sleep(250);
        servoBlue.setPosition(ARM_HOME);

    }

    private void extendRuller() throws InterruptedException {
        servoParking.setPower(1.0);
        sleep(4500);
        servoParking.setPower(0.0);
    }

    public double getDistance() {
        return sensorRange.getDistance(DistanceUnit.CM);
    }

    /* Make Sample Red */

    public void makeSampleRed() throws InterruptedException {

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
                                //moveRight(1);
                                while(sensorRange.getDistance(DistanceUnit.CM) > 5) {
                                    //wait
                                    moveForward();
                                }
                                stop();
                                //wait
                                sleep(100);
                                takeSkyStoneRed();
                                found = true;
                                position = "center";
                            } else {
                                //Position right
                                while(sensorRange.getDistance(DistanceUnit.CM) > 5) {
                                    //wait
                                    moveForward();
                                }
                                stop();
                                //wait
                                sleep(100);
                                moveRight(18);
                                takeSkyStoneRed();
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
            moveForward();
            while(sensorRange.getDistance(DistanceUnit.CM) >= 5) {
                //wait
                sleep(30);
            }
            stop();
            sleep(500);
            moveLeft(5);
            takeSkyStoneRed();
            position = "left";
        }

        sleep(500);

        switch (position) {
            case "left" :
                strafeBackward(15);
                navigateRight(140);
                leaveSkyStoneRed();
                sleep(250);
                //Ma intorc dupa celalalt
                navigateLeft(190);
                moveForward();
                while(sensorRange.getDistance(DistanceUnit.CM) > 5) {
                    //wait
                }
                stop();
                sleep(100);
                takeSkyStoneRed();
                sleep(250);
                strafeBackward(15);
                navigateRight(195);
                leaveSkyStoneRed();
                sleep(250);
                break;

            case "right" :
                strafeBackward(15);
                navigateRight(110);
                leaveSkyStoneRed();
                //Ma intorc dupa celalalt
                sleep(250);
                navigateLeft(170);
                moveForward();
                while(sensorRange.getDistance(DistanceUnit.CM) > 5) {
                    //wait
                }
                stop();
                sleep(100);
                takeSkyStoneRed();
                sleep(100);
                strafeBackward(15);
                navigateRight(175);
                leaveSkyStoneRed();
                sleep(250);
                break;

            case "center" :
                strafeBackward(15);
                navigateRight(130);
                leaveSkyStoneRed();
                sleep(250);
                //Ma intorc dupa celalalt
                navigateLeft(190);
                moveForward();
                while(sensorRange.getDistance(DistanceUnit.CM) > 5) {
                    //wait
                }
                stop();
                sleep(100);
                takeSkyStoneRed();
                sleep(100);
                strafeBackward(15);
                navigateRight(195);
                leaveSkyStoneRed();
                sleep(250);
                break;
        }

    }

    /* Make Sample Blue */

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
                                //moveRight(1);
                                moveForward();
                                while(sensorRange.getDistance(DistanceUnit.CM) > 5) {
                                    //wait
                                }
                                stop();
                                //wait
                                sleep(100);
                                takeSkyStoneBlue();
                                found = true;
                                position = "center";
                            } else {
                                //Position right
                                moveForward();
                                while(sensorRange.getDistance(DistanceUnit.CM) > 5) {
                                    //wait
                                }
                                stop();
                                //wait
                                sleep(100);
                                moveRight(18);
                                takeSkyStoneBlue();
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
            moveForward();
            while(sensorRange.getDistance(DistanceUnit.CM) > 5) {
                //wait
            }
            stop();
            sleep(100);
            moveLeft(5);
            takeSkyStoneRed();
            position = "left";
        }

        sleep(500);

        switch (position) {
            case "left" :
                strafeBackward(15);
                navigateLeft(110);
                leaveSkyStoneRed();
                sleep(250);
                //Ma intorc dupa celalalt
                navigateRight(170);
                moveForward();
                while(sensorRange.getDistance(DistanceUnit.CM) > 5) {
                    //wait
                }
                stop();
                sleep(100);
                takeSkyStoneRed();
                sleep(250);
                strafeBackward(15);
                navigateLeft(175);
                leaveSkyStoneRed();
                sleep(250);
                break;

            case "right" :
                strafeBackward(15);
                navigateLeft(140);
                leaveSkyStoneRed();
                //Ma intorc dupa celalalt
                sleep(250);
                navigateRight(190);
                moveForward();
                while(sensorRange.getDistance(DistanceUnit.CM) > 5) {
                    //wait
                }
                stop();
                sleep(100);
                takeSkyStoneRed();
                sleep(100);
                strafeBackward(15);
                navigateLeft(195);
                leaveSkyStoneRed();
                sleep(250);
                break;

            case "center" :
                strafeBackward(15);
                navigateLeft(130);
                leaveSkyStoneRed();
                sleep(250);
                //Ma intorc dupa celalalt
                navigateRight(190);
                moveForward();
                while(sensorRange.getDistance(DistanceUnit.CM) > 5) {
                    //wait
                }
                stop();
                sleep(100);
                takeSkyStoneRed();
                sleep(100);
                strafeBackward(15);
                navigateLeft(195);
                leaveSkyStoneRed();
                sleep(250);
                break;
        }

    }

    /* Platforma */

    private void catchPlate() throws InterruptedException {
        servoPlateLeft.setPosition(0.62);
        servoPlateRight.setPosition(0.62);
        sleep(500);
    }

    private void releasePlate() throws InterruptedException {
        servoPlateLeft.setPosition(0);
        servoPlateRight.setPosition(0);
        sleep(500);
    }

    public void repositionRed() throws InterruptedException {
        //Navigare
        moveLeft(52);

        sleep(250);

        catchPlate();

        moveRight(55);

        sleep(250);

        turn90Right(65);

        releasePlate();

        moveLeft(10);

        //Park

        strafeForward(32);

        sleep(250);

        moveRight(97);
    }

    public void repositionRedRuleta() throws InterruptedException {
        //Navigare
        moveLeft(52);

        sleep(250);

        catchPlate();

        moveRight(55);

        sleep(250);

        turn90Right(65);

        releasePlate();

        moveLeft(15);

        //Park
        //Turn 180
        moveRight(20);

        sleep(250);

        turn90Left(100);

        strafeBackward(38);

        extendRuller();
    }

    public void repositionBlue() throws InterruptedException {
        //Navigare
        moveLeft(52);

        sleep(250);

        catchPlate();

        moveRight(55);

        sleep(250);

        turn90Left(65);

        releasePlate();

        moveLeft(10);

        //Park

        strafeBackward(38);

        sleep(250);

        moveRight(97);
    }

    public void repositionBlueRuleta() throws InterruptedException {
        //Navigare
        moveLeft(52);

        sleep(250);

        catchPlate();

        moveRight(55);

        sleep(250);

        turn90Left(65);

        releasePlate();

        moveLeft(20);

        //Park
        //Turn 180

        moveRight(15);

        sleep(250);

        turn90Right(105);

        strafeForward(38);

        extendRuller();
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