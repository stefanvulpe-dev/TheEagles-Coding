package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Manual", group="Linear Opmode")

public class Manual extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Eagle eagle = new Eagle();
    private String buttonState = "Not Pressed";

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        eagle.initHardware(hardwareMap);

        while (isStarted() == false && isStopRequested() == false) {
            telemetry.addData("some key", "some data");
            telemetry.update();
            sleep(200);
        }

        //waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            switch (buttonState) {
                case "Pressed":
                    telemetry.addData("ButtonState: ", "pressed");
                    if(gamepad1.start) {
                        buttonState = "Not Pressed";
                        sleep(200);
                    }
                    eagle.constructionMode(gamepad1.dpad_up, gamepad1.dpad_down, gamepad1.dpad_left, gamepad1.dpad_right);
                    eagle.moveLift(gamepad2.dpad_up, gamepad2.dpad_down);
                    eagle.moveArm(gamepad2.right_bumper, gamepad2.left_bumper);
                    eagle.actionServoClaw(gamepad2.a, gamepad2.y);
                    eagle.actionServoPlate(gamepad1.right_trigger, gamepad1.left_trigger);
                    eagle.actionServoParking(gamepad2.right_trigger, gamepad2.left_trigger);
                break;
                case "Not Pressed":
                    if(gamepad1.start) {
                        buttonState = "Pressed";
                        sleep(200);
                    }
                    telemetry.addData("ButtonState: ", "not pressed");
                    eagle.manualMove(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
                    eagle.moveLift(gamepad2.dpad_up, gamepad2.dpad_down);
                    eagle.intake(gamepad1.right_bumper, gamepad1.left_bumper);
                    eagle.moveArm(gamepad2.right_bumper, gamepad2.left_bumper);
                    eagle.actionServoClaw(gamepad2.a, gamepad2.y);
                    eagle.actionServoPlate(gamepad1.right_trigger, gamepad1.left_trigger);
                    eagle.actionServoTeamMarker(gamepad2.x,gamepad2.b);
                    eagle.actionServoParking(gamepad2.right_trigger, gamepad2.left_trigger);
                break;
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
