package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Bruhtonomous",group = "SkyStone")
public class BasicAuto extends BaseAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {
        inithardware(true);
        waitForStart();



        testdrive(0.5,30,robot.frontLeft);
        testdrive(0.5,30,robot.frontRight);
        testdrive(0.5,30,robot.backRight);
        testdrive(0.5,30,robot.backLeft);



    }

}
