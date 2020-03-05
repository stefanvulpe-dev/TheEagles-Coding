package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="BuildingBlue", group="Linear Opmode")

public class BuildingBlue extends LinearOpMode {

    private Eagle eagle = new Eagle();

    @Override
    public void runOpMode() throws InterruptedException {

        //Init Hardware
        eagle.initHardware(hardwareMap);


        while (isStarted() == false && isStopRequested() == false) {
            telemetry.addData("some key", "some data");
            telemetry.update();
            sleep(200);
        }

        if(opModeIsActive()) {

            eagle.strafeBackward(15);
            eagle.repositionBlue();

        }


    }
}
