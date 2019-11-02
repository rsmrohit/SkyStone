package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "GeneralAuto",group = "SkyStone")
public class GeneralAuto extends BaseAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {

        //Initialize the hardware using BaseAutonomous Function
        inithardware(true);

        telemetry.addData("program","initing");
        telemetry.update();


        //Wait for the start button to be pressed
        waitForStart();

        encoderMecanumDrive(DRIVE_SPEED,5.5,1,-1,0);
        encoderMecanumDrive(DRIVE_SPEED,5.3,1,0,1);
        sleep(500);
        rightTurn();
        encoderMecanumDrive(DRIVE_SPEED,1.2,1,0,1);
        succ(1000);
        encoderMecanumDrive(DRIVE_SPEED,1,1,0,-1);
        leftTurn();
        sleep(200);
        encoderMecanumDrive(DRIVE_SPEED,20,2,-1,0);
        encoderMecanumDrive(DRIVE_SPEED,6.5,1,0,1);
        spit();






    }
}
