package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutonomBuildingSiteBlue", group = "Autonomous")

public class AutonomBuildingSiteBlue extends LinearOpMode {

    private Eagle eagle = new Eagle();



    @Override
    public void runOpMode() {

        // Initialization
        telemetry.addData("Init ", "started");
        telemetry.update();

        //eagle.initTF(hardwareMap);

        telemetry.addData("Vuforia & Tfod", "initialized");
        telemetry.update();

        eagle.initHardware(hardwareMap);

        telemetry.addData("Robot", "initialized");
        telemetry.update();


        telemetry.addData(">", "Press Play to start");
        telemetry.update();



        waitForStart();


        //move pana vad 3 blcuri
        eagle.move(-0.35);
        sleep(2500);
        eagle.move(0);
        //eagle.searchSkystone();


        // Skystone found, time is up or stop was requested.
        //eagle.tfod.shutdown();
        // Pause to let driver station to see last telemetry.
        //sleep(2000);

    }



}