package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name = "Bruhtonomous",group = "SkyStone")
public class Testing extends BaseAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {

        //Initialize the hardware using BaseAutonomous Function
        inithardware(false);
        telemetry.addData("at position", robot.clamper.getPosition());

        //Wait for the start button to be pressed
        waitForStart();

        gyroCurve(0.5,-45,0,1);



//        dumbencoderMecanumDrive(1,130, 4, 0,1,false);

    }

}
