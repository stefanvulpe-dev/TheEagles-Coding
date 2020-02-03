package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="AutonomLoadingBlue", group="Linear Opmode")

public class AutonomLoadingBlue extends LinearOpMode {

    private Eagle eagle = new Eagle();

    @Override
    public void runOpMode() throws InterruptedException {

        //Init Hardware
        eagle.initHardware(hardwareMap);

        //Init TF
        eagle.initTF(hardwareMap);

        waitForStart();

        if(opModeIsActive()) {
            //Scan stones
            eagle.strafeForward(55);

            //make Sample
            eagle.makeSampleBlue();

            //Park
            eagle.moveLeft(10);

            //End
        }

        eagle.TF_shutdown();

    }
}
