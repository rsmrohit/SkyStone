package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name = "Bruhtonomous",group = "SkyStone")
public class Testing extends BaseAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {

        //Initialize the hardware using BaseAutonomous Function
        inithardware(false);

        //Wait for the start button to be pressed
        waitForStart();

//        for (double i =0;i< 1;i= i + 0.1){
//
//            robot.clamper.setPosition(i);
//            telemetry.addData("at position", robot.clamper.getPosition());
//            telemetry.update();
//            sleep(1000);
//
//        }


        testdrive(1.0,100,robot.frontRight);
        testdrive(1.0,100,robot.frontLeft);
        testdrive(1.0,100,robot.backRight);
        testdrive(1.0,100,robot.backLeft);


    }

}
