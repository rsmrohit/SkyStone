package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name = "Bruhtonomous",group = "SkyStone")
public class Testing extends BaseAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {

        //Initialize the hardware using BaseAutonomous Function

        inithardware(false);

//        telemetry.addData("program", "intialized");
//        telemetry.addData("horizontal encoder",robot.horizontalSlider.getCurrentPosition());
//        telemetry.addData("vertical encoder", robot.verticalSlider.getCurrentPosition());
//        telemetry.update();
//        robot.clamper.setPosition(0.15);
//
//        //Wait for the start button to be pressed
          waitForStart();
        dumbencoderMecanumDrive(0.9,40,5,-1,1,false);

//
//        encoderMecanumDrive(0.9,35,5,0,1);
//        String s = vuforiaJoint(haddi,buddi);
//        telemetry.addData("location", s);
//        telemetry.update();
//        sleep(2000);



    }

}
