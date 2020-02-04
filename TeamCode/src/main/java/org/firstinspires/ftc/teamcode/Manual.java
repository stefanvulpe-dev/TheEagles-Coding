package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Manual", group="Linear Opmode")

public class Manual extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Eagle eagle = new Eagle();

    @Override
    public void runOpMode() {

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

            eagle.manualMove(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
            eagle.moveLift(gamepad2.dpad_up, gamepad2.dpad_down);
            eagle.intake(gamepad1.right_bumper, gamepad1.left_bumper);
            eagle.moveArm(gamepad2.right_bumper, gamepad2.left_bumper);
            eagle.actionServoClaw(gamepad2.a, gamepad2.y);
            eagle.actionServoPlate(gamepad2.x, gamepad2.b);
            //eagle.actionServoTeamMarker(gamepad2.x,gamepad2.b);
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
