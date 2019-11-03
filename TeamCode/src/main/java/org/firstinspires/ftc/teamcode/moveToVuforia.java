package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "stinky",group = "SkyStone")
public class moveToVuforia extends BaseAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {

        //Initialize the hardware using BaseAutonomous Function
        inithardware(true);

        telemetry.addData("program","initing");
        telemetry.update();


        //Wait for the start button to be pressed
        waitForStart();

        encoderMecanumDrive(DRIVE_SPEED,2.5,1,1,0);







    }
}
