package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;



@Autonomous(name = "Bruhtonomous",group = "SkyStone")
public class BasicAuto extends BaseAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {
        inithardware(true);
        waitForStart();




        encoderMecanumDrive(DRIVE_SPEED,60,4,-1,0);
        encoderMecanumDrive(DRIVE_SPEED,50,4,0,1);
        String location = vuforiaJoint(haddi,buddi);
        telemetry.addData("location",location);
        telemetry.addData("runtime", runtime.seconds());
        telemetry.update();
        sleep(1000);



    }

}
