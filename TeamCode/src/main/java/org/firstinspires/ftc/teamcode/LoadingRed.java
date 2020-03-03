package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="LoadingRed", group="Linear Opmode")

public class LoadingRed extends LinearOpMode {

    private Eagle eagle = new Eagle();

    @Override
    public void runOpMode() throws InterruptedException {

        //Init Hardware
        eagle.initHardware(hardwareMap);

        //Init TF
        eagle.initTF(hardwareMap);

        while (isStarted() == false && isStopRequested() == false) {
            telemetry.addData("some key", "some data");
            telemetry.update();
            sleep(200);
        }

        if(opModeIsActive()) {
            //Scan stones
            eagle.strafeForward(55);

            //make Sample
            eagle.makeSampleRed();

            //Park
            eagle.moveLeft(10);

            //End
        }

        eagle.TF_shutdown();

    }
}