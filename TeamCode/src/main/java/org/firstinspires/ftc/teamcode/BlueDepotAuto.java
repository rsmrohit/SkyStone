package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BlueDepotAuto",group = "SkyStone")
public class BlueDepotAuto extends BaseAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {

        //Initialize the hardware using BaseAutonomous Function
        inithardware(false);

        telemetry.addData("program","initing");
        telemetry.update();


        //Wait for the start button to be pressed
        waitForStart();

        String location = "Center";


        if (location.equals("Center")){
            //first block
            //strafe and move forward for block setup
            encoderMecanumDrive(0.75,32.5,1,1,0);
            encoderMecanumDrive(0.9,60,5,0,1);
            gyroTurn(0.9,52);
            succ();
            //move to collect block
            encoderMecanumDrive(0.75,75,4,1,0.95);
            //fine tune grab block
            encoderMecanumDrive(1,22.5,4,-0.819152,-0.573576);
            //move back to be able to move to foundation
            encoderMecanumDrive(1,68,4,0,-1);
            succstop();
            robot.clamper.setPosition(0.15);
            //turn towards foundation
            gyroTurn(0.9,90);
            telemetry.addData("angle", robot.realgyro.getIntegratedZValue());
            telemetry.update();
            encoderMecanumDrive(1,90,10,0,-1);
            //move to foundation
            bruh();
            robot.clamper.setPosition(0.03);


            //coming back to get second block
            dumbencoderMecanumDrive(1,180, 4, 0,1,false);
            encoderMecanumDrive(1,58,10,1,0);
            succ();
            encoderMecanumDrive(0.7,20,10,0,1);
            gyroTurn(0.9,-90);
            encoderMecanumDrive(1,130,10,-1,-1);
            succstop();
            robot.clamper.setPosition(0.15);
            encoderMecanumDrive(1,130,10,0,-1);
            bruhbuddi();
            robot.clamper.setPosition(0.03);
            //move to park
            dumbencoderMecanumDrive(1,70, 4, 0,1,false);


        }


    }
}
