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

        gyroTurnAndMove(0.5,-90,0.5,0);
        release();
        //should move in a curve to the right
//        gyroCurve(0.5,-45,0,0.5);
//
//
//        //should turn in place
//        gyroCurve(0.5,0,0,0);

//        dumbencoderMecanumDrive(1,130, 4, 0,1,false);

    }

}
