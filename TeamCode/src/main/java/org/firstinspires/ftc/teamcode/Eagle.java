package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaSkyStone;


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

    private HardwareMap hwMap;

    private static final int MOTOR_TICK_COUNTS = 1120;
    private static final double ARM_MAX_RANGE = 1.0d;
    private static final double ARM_MIN_RANGE = 0.0d;
    private static final double ARM_HOME = 0.0d;
    private static final double ARM_SPEED = 0.005;
    private double servoPosition = ARM_HOME;

    //Constructor
    public Eagle() {

    }


    public void init(HardwareMap hardwareMap) {
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

        //Set Mode
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        servoLeft.setPosition(ARM_HOME);
        servoRight.setPosition(ARM_HOME);
        servoClaw.setPosition(ARM_HOME);

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
            motorLift.setPower(0.5);
        } else if(power2) {
            motorLift.setPower(-0.5);
        } else {
            motorLift.setPower(0.0);
        }
    }

    public void intake(boolean power1, boolean power2) {
        if(power1) {
            intakeRight.setPower(0.75);
            intakeLeft.setPower(0.75);
        } else if(power2) {
            intakeRight.setPower(-0.75);
            intakeLeft.setPower(-0.75);
        } else {
            intakeRight.setPower(0);
            intakeLeft.setPower(0);
        }
    }

    public void moveArm(double power1, double power2) {
        if(power1 > 0) {
            servoPosition += ARM_SPEED;
        } else if(power2 > 0) {
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
            servoClaw.setPosition(0.5);
        } else if(power2) {
            servoClaw.setPosition(0.0);
        } else {
            servoClaw.setPosition(servoClaw.getPosition());
        }
    }

    //Functii autonom

    public void move(double distance) {
        //Set Mode
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Reset encoders
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double circumference = Math.PI * 3.93701;
        double rotationsNeeded = distance/circumference;
        int target = (int)(MOTOR_TICK_COUNTS * rotationsNeeded);

        leftFrontDrive.setTargetPosition(target);
        leftBackDrive.setTargetPosition(target);
        rightFrontDrive.setTargetPosition(target);
        rightBackDrive.setTargetPosition(target);

        leftFrontDrive.setPower(.5);
        leftBackDrive.setPower(.5);
        rightFrontDrive.setPower(.5);
        rightBackDrive.setPower(.5);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while(leftFrontDrive.isBusy() || leftBackDrive.isBusy() || rightFrontDrive.isBusy() || rightBackDrive.isBusy()) {
            //wait
        }

        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);

    }


}
