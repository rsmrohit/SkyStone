package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "blueBuildingSiteAuto",group = "SkyStone")
public class blueBuildingSiteAuto extends BaseAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {
        inithardware(false);
        waitForStart();
        encoderMecanumDrive(DRIVE_SPEED,68,2,0,-1);
        encoderMecanumDrive(DRIVE_SPEED,50,2,1,0);
        encoderMecanumDrive(0.4,10,2,0,-1);
        grab();
        encoderMecanumDrive(DRIVE_SPEED,47,2,0,1);
        smallgrab();
        encoderMecanumDrive(DRIVE_SPEED,47,2,0,1);
        sleep(300);
        release();
        encoderMecanumDrive(DRIVE_SPEED,100,2,-1,0);




    }

}
