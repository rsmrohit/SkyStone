package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "redBuildingSiteAuto",group = "SkyStone")
public class redBuildingSiteAuto extends BaseAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {
        inithardware(false);
        waitForStart();
        release();
        encoderMecanumDrive(DRIVE_SPEED,65,2,0,-1);
        encoderMecanumDrive(DRIVE_SPEED,30,2,-1,0);
        encoderMecanumDrive(0.5,22.5,2,0,-1);
        grab();
        sleep(300);

        encoderMecanumDrive(1.0,50,5,0,1);
        gyroCurve(0.8,-90,0,0.5);
        release();
        encoderMecanumDrive(0.6,50,5,0,-1);

        //go sideways into the wall
//        encoderMecanumDrive(1,5,1,0,1);
        gyroTurn(0.7,-90);
        encoderMecanumDrive(1.0,50,2,-1,0);

        encoderMecanumDrive(1.0,80,4,0,1);


    }

}