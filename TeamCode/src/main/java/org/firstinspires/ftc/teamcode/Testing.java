package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;



@Autonomous(name = "Bruhtonomous",group = "SkyStone")
public class Testing extends BaseAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {

        //Initialize the hardware using BaseAutonomous Function
        inithardware(false);

        telemetry.addData("program","initing");
        telemetry.update();


        //Wait for the start button to be pressed
        waitForStart();

        encoderMecanumDrive(DRIVE_SPEED,44,4,-1,0);
        encoderMecanumDrive(DRIVE_SPEED,50,4,0,-1);


        String location = vuforiaJointo(haddi,buddi);
        telemetry.addData("location",location);
        telemetry.update();
        sleep(1000);


    }

}
