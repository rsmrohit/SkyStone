package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "small auto",group = "SkyStone")
public class smallauto extends BaseAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {
        inithardware(false);
        waitForStart();
        double FRONT = Math.toRadians(90);
        goToPosition(0,10*COUNTS_PER_INCH,0.5, 2.5*COUNTS_PER_INCH, FRONT,0.4, Math.toRadians(30), 0.5);
    }

}
