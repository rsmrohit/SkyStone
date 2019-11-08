package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RedDepotAuto",group = "SkyStone")
public class RedDepotAuto extends BaseAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {

        //Initialize the hardware using BaseAutonomous Function
        inithardware(true);

        telemetry.addData("program","initing");
        telemetry.update();


        //Wait for the start button to be pressed
        waitForStart();

//        encoderMecanumDrive(DRIVE_SPEED,5.5,1,-1,0);
//        encoderMecanumDrive(DRIVE_SPEED,5.3,1,0,1);
//        sleep(500);
//        rightTurn();
//        encoderMecanumDrive(DRIVE_SPEED,1.2,1,0,1);
//        succ();
//        encoderMecanumDrive(DRIVE_SPEED,1,1,0,-1);
//        leftTurn();
//        sleep(200);
//        encoderMecanumDrive(DRIVE_SPEED,20,2,-1,0);
//        encoderMecanumDrive(DRIVE_SPEED,6.5,1,0,1);
//        spit();

        encoderMecanumDrive(DRIVE_SPEED,40,2,0,1);
        encoderMecanumDrive(DRIVE_SPEED,30,2,1,0);
        String location = vuforiaJoint(haddi, buddi);
        telemetry.addData("location",location);
        telemetry.update();
        sleep(1000);






    }
}
