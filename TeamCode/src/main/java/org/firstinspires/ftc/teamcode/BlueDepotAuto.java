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
            encoderMecanumDrive(0.55,31,1,1,0);
            sleep(500);
            encoderMecanumDrive(1,60,5,0,1);
            sleep(500);
            gyroTurn(0.5,52);
            sleep(500);
            succ();
            encoderMecanumDrive(0.5,55,4,1,1);

            // NOT EDITED FROM redDepotAuto YET
//            succstop();
//            encoderMecanumDrive(0.8,50,4,0,-1);
//            gyroTurn(0.8,-90);
//            dumbencoderMecanumDrive(1,130, 4, 0,-1,true);
//            encoderMecanumDrive(0.8,10, 4, 1,0);
//            robot.clamper.setPosition(0.03);

        }
















//        encoderMecanumDrive(DRIVE_SPEED,50,4,-1,0);
//        encoderMecanumDrive(DRIVE_SPEED,50,4,0,-1);
//
//
//        String location = vuforiaJointo(haddi,buddi);
//        telemetry.addData("location",location);
//        telemetry.update();
//
//        if (location.equals("Right")){
//            encoderMecanumDrive(DRIVE_SPEED,5,1,0,1);
//            gyroTurn(TURN_SPEED,60);
//            encoderMecanumDrive(DRIVE_SPEED,6,1,-0.8660254,-0.5);
//            succ();
//            encoderMecanumDrive(0.2,67,10,-0.5,0.8660254);
//            succstop();
//            gyroTurn(0.4,90);
//            encoderMecanumDrive(1.0,47,10,0,-1);
//            encoderMecanumDrive(1.0,220,10,1,0);
//            spit();
//            encoderMecanumDrive(DRIVE_SPEED,50,2,-1,0);
//
//
//        } else if (location.equals("Left")){
//
//
//            encoderMecanumDrive(DRIVE_SPEED,5,1,0,1);
//            gyroTurn(TURN_SPEED,135);
//            //encoderMecanumDrive(DRIVE_SPEED,8,2,0.93969,0.34202);
//            succ();
//            encoderMecanumDrive(0.2,67,10,1,1);
//            succstop();
//            gyroTurn(0.4,90);
//            encoderMecanumDrive(1.0,47,10,0,-1);
//            encoderMecanumDrive(1.0,220,10,1,0);
//            spit();
//            encoderMecanumDrive(DRIVE_SPEED,50,2,-1,0);
//
//
//        }else if (location.equals("Center")){
//
//            gyroTurn(TURN_SPEED,90);
//            succ();
//            encoderMecanumDrive(0.2,67,10,0,1);
//            succstop();
//            encoderMecanumDrive(1.0,47,10,0,-1);
//            encoderMecanumDrive(1.0,220,10,1,0);
//            spit();
//            encoderMecanumDrive(DRIVE_SPEED,50,2,-1,0);
//        }





    }
}
