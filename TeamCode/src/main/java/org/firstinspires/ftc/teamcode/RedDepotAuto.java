package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RedDepotAuto",group = "SkyStone")
public class RedDepotAuto extends BaseAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {

        //Initialize the hardware using BaseAutonomous Function
        inithardware(false);

        telemetry.addData("program","initialized");
        telemetry.update();


        //Wait for the start button to be pressed
        waitForStart();

        //This line is used only if we align it with the edge of the mat block
        //encoderMecanumDrive(0.5,8.75,1,1,0);

        // String location = vuforiaJoint(haddi,buddi);
        String location = "Center";



        if (location.equals("Center")){
            encoderMecanumDrive(0.8,26,1,-1,0);
            sleep(500);
            encoderMecanumDrive(1,60,5,0,1);
            sleep(500);
            gyroTurn(0.5,-45);
            sleep(500);
            succ();
            encoderMecanumDrive(0.5,50,4,-1,1);

        }












































//        encoderMecanumDrive(DRIVE_SPEED,60,4,-1,0);
//        encoderMecanumDrive(DRIVE_SPEED,50,4,0,1);
//
//

//
//        if (location.equals("Right")){
//            gyroTurn(TURN_SPEED,45);
//            //encoderMecanumDrive(DRIVE_SPEED,8,2,0.93969,0.34202);
//            succ();
//            encoderMecanumDrive(0.2,67,10,-1,1);
//            succstop();
//            gyroTurn(0.4,90);
//            encoderMecanumDrive(1.0,50,10,0,-1);
//            encoderMecanumDrive(1.0,220,10,-1,0);
//            spit();
//            encoderMecanumDrive(DRIVE_SPEED,55,2,1,0);
//
//        } else if (location.equals("Left")){
//            gyroTurn(TURN_SPEED,120);
//            encoderMecanumDrive(DRIVE_SPEED,6,1,0.8660254,-0.5);
//
//            succ();
//
//            encoderMecanumDrive(0.2,67,10,0.5,0.8660254);
//
//            succstop();
//            gyroTurn(0.4,90);
//            encoderMecanumDrive(1.0,50,10,0,-1);
//            encoderMecanumDrive(1.0,220,10,-1,0);
//            spit();
//            encoderMecanumDrive(DRIVE_SPEED,50,2,1,0);
//        }else if (location.equals("Center")){
//            gyroTurn(TURN_SPEED,90);
//            succ();
//            encoderMecanumDrive(0.2,67,10,0,1);
//            succstop();
//            encoderMecanumDrive(1.0,50,10,0,-1);
//            encoderMecanumDrive(1.0,220,10,-1,0);
//            spit();
//            encoderMecanumDrive(DRIVE_SPEED,55,2,1,0);
//        }





    }
}
