package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Autonomous(name = "BlueDepotAuto",group = "SkyStone")
public class BlueDepotAuto extends BaseAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {

        //Initialize the hardware using BaseAutonomous Function
        inithardware(false);
        telemetry.clear();
        telemetry.addData("program","initialized");
        telemetry.update();


        //Wait for the start button to be pressed
        waitForStart();
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        encoderMecanumDrive(0.9,35,5,0,1);

        String location = vuforiaJointo(haddi,buddi);
        telemetry.addData("location", location);
        telemetry.update();


        if (location.equals("Center")){
            //first block
            //strafe and move forward for block setup
            encoderMecanumDrive(0.75,60,1,1,0);
            encoderMecanumDrive(0.9,25,5,0,1);
            gyroTurn(0.9,52,2);
            succ();
            //move to collect block
            encoderMecanumDrive(0.75,75,4,1,0.95);
            //fine tune grab block
            encoderMecanumDrive(1,18,4,-0.819152,-0.573576);
            //move back to be able to move to foundation
            encoderMecanumDrive(1,68,4,0,-1);
            succstop();

            robot.clamper.setPosition(0.15);
            //turn towards foundation
            gyroTurn(0.9,90,2);

            encoderMecanumDrive(1,85,10,0,-1);
            //move to foundation
            bruh();
            robot.clamper.setPosition(0.03);


            //coming back to get second block
            dumbencoderMecanumDrive(1,170, 4, 0,1,false);
            encoderMecanumDrive(1,60,10,1,0);
            succ();
            encoderMecanumDrive(0.5,20,10,0,1);
            encoderMecanumDrive(1,130,10,-1,-1);
            succstop();
            robot.clamper.setPosition(0.15);
            encoderMecanumDrive(1,130,9,0,-1);
            bruhbuddi();
            //move to park
            dumbencoderMecanumDrive(1,70, 4, 0,1,false);


        }else if(location.equals("Left")){
            encoderMecanumDrive(0.75,24,1,1,0);
            encoderMecanumDrive(0.9,25,5,0,1);
            gyroTurn(0.9,52,2);
            succ();
            //move to collect block
            encoderMecanumDrive(0.75,75,4,1,0.95);
            //fine tune grab block
            encoderMecanumDrive(1,18,4,-0.819152,-0.573576);
            //move back to be able to move to foundation
            encoderMecanumDrive(1,68,4,0,-1);
            succstop();
            robot.clamper.setPosition(0.15);
            //turn towards foundation
            gyroTurn(0.9,88,2);
            encoderMecanumDrive(1,105,10,0,-1);
            //move to foundation
            bruh();
            robot.clamper.setPosition(0.03);


            //coming back to get second block
            dumbencoderMecanumDrive(1,190, 4, 0,1,false);
            encoderMecanumDrive(1,50,10,1,0);

            succ();
            encoderMecanumDrive(0.4,20,10,1,1);

            encoderMecanumDrive(1,130,8,-1,-1);
            succstop();
            robot.clamper.setPosition(0.15);
            encoderMecanumDrive(1,130,10,0,-1);
            bruhbuddi();

            //move to park
            dumbencoderMecanumDrive(1,70, 4, 0,1,false);



        }else{
            //first block
            //strafe and move forward for block setup
            encoderMecanumDrive(0.75,8,1,-1,0);

            encoderMecanumDrive(0.9,25,5,0,1);
            gyroTurn(0.9,-52,2);
            succ();
            //move to collect block
            encoderMecanumDrive(0.75,75,4,-1.0,0.95);

            //fine tune grab block
            encoderMecanumDrive(1,20,4,0.819152,-0.573576);

            gyroTurn(0.9,0,2);
            encoderMecanumDrive(1,45,10,0,-1);
            gyroTurnAndMove(0.9,88,0.5,-90);
            succstop();
            robot.clamper.setPosition(0.15);
            //move back to be able to move to foundation
            encoderMecanumDrive(1,120,4,0,-1);

            //move to foundation
            bruh();
            robot.clamper.setPosition(0.03);

            //second block

            //coming back to get second block
            dumbencoderMecanumDrive(1,160, 4, 0,1,false);

            //strafing sideways
            encoderMecanumDrive(1,70,10,1,0);
            succ();
            encoderMecanumDrive(0.5,25,10,1,1);

            encoderMecanumDrive(1,123,10,-1,-1);
            succstop();
            robot.clamper.setPosition(0.15);
            encoderMecanumDrive(1,125,10,0,-1);
            bruhbuddi();


            //move to park
            dumbencoderMecanumDrive(1,70, 4, 0,1,false);


        }


    }
}
