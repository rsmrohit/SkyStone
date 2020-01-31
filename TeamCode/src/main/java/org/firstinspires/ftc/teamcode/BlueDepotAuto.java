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
        robot.clamper.setPosition(0.2);
        //turn towards foundation
        gyroTurn(0.9,90);
        telemetry.addData("angle", robot.realgyro.getIntegratedZValue());
        telemetry.update();
        encoderMecanumDrive(1,90,10,0,-1);
        //move to foundation
        bruh();
        robot.clamper.setPosition(0.03);

        //second block
        //coming back to get second block
        dumbencoderMecanumDrive(1,183.5, 4, 0,1,false);
        //turn to collect
        gyroTurn(0.9,48);
        telemetry.addData("angle", robot.realgyro.getIntegratedZValue());
        telemetry.update();
        sleep(200);
        //start sucking
        succ();
        //strafe to block pt 1
        encoderMecanumDrive(0.75,77,4,0.93,1);
        //drive straight to separate blocks for collection
        encoderMecanumDrive(0.9,30,4,0,1);
        //strafe to block pt 2
        encoderMecanumDrive(1,22.5,4,-0.819152,-0.5/*73576*/);
        sleep(100);
        //move back to original position
        encoderMecanumDrive(1,65,4,0,-1);
        succstop();
        robot.clamper.setPosition(0.2);
        //turn to straight position
        gyroTurn(1,85);
        telemetry.addData("angle", robot.realgyro.getIntegratedZValue());
        telemetry.update();
        //move to foundation
        encoderMecanumDrive(1,135, 4, 0,-1);
        bruh();
        robot.clamper.setPosition(0.03);
        //move to park
        dumbencoderMecanumDrive(1,70, 4, 0,1,false);
















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
