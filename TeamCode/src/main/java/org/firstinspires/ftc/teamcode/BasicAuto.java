package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Bruhtonomous",group = "SkyStone")
public class BasicAuto extends BaseAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        vuforiaJoint();

    }

}
