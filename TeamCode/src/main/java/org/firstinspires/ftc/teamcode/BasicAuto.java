package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Bruhtonomous",group = "SkyStone")
public class BasicAuto extends BaseAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {
        inithardware(true);
        waitForStart();
//        String location = vuforiaJoint(haddi, buddi);
//        telemetry.addData("location",location);
//        telemetry.update();
//        sleep(1000);
        grab();
        release();
        grab();
        release();


    }

}
