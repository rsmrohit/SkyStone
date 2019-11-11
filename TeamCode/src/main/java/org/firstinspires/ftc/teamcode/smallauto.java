package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "small auto",group = "SkyStone")
public class smallauto extends BaseAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {
        inithardware(false);
        waitForStart();
        encoderMecanumDrive(0.5,1,1,0,1);
        encoderMecanumDrive(0.5,15,1,-1,0);
    }

}
