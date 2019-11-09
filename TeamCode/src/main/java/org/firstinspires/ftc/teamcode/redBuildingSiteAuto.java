package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "redBuildingSiteAuto",group = "SkyStone")
public class redBuildingSiteAuto extends BaseAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {
        inithardware(false);
        waitForStart();

        encoderMecanumDrive(DRIVE_SPEED,75,2,1,0);
        encoderMecanumDrive(DRIVE_SPEED,22,2,0,-1);
        encoderMecanumDrive(0.5,22.5,2,1,0);
        grab();
        gyroTurn(0.4,2);
        //encoderMecanumDrive(0.4,10,1,0,1);
        encoderMecanumDrive(1.0,200,5,-1,0);
        //reverse turn if needed to become straight
        //gyroTurn(0.4,0);
        //encoderMecanumDrive(1.0,150,5,-1,0);
        sleep(300);
        release();
        encoderMecanumDrive(DRIVE_SPEED,100,2,0,1);








    }

}