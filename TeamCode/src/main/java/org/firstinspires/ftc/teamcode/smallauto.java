package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "small auto",group = "SkyStone")
public class smallauto extends BaseAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {
        inithardware(false);
        waitForStart();
        goToPosition(0,10*COUNTS_PER_INCH,0.5,Math.toRadians(90),0.4,Math.toRadians(30), 0.5);
    }

}
