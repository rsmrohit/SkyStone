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
        String location = "Center";

//        encoderMecanumDrive(DRIVE_SPEED, 25, 4,0,1);
//
//        String location = vuforiaJoint(haddi,buddi);
//        telemetry.addData("location", location);
//        telemetry.update();

        if (location.equals("Center")){
            encoderMecanumDrive(0.75,33.5,1,-1,0);
            encoderMecanumDrive(0.9,60,5,0,1);
            gyroTurn(0.9,-52);
            succ();
            encoderMecanumDrive(0.75,55,4,-1,0.955);
            encoderMecanumDrive(1,15,4,0.819152,-0.573576);
            encoderMecanumDrive(1,68,4,0,-1);
            succstop();
            robot.clamper.setPosition(0.2);
            gyroTurn(0.9,-90);
            telemetry.addData("angle", robot.realgyro.getIntegratedZValue());
            telemetry.update();
            encoderMecanumDrive(1,135,10,0,-1);
            bruh();
            robot.clamper.setPosition(0.03);

            sleep(30000);


            dumbencoderMecanumDrive(1,230, 4, 0,1,false);
            gyroTurn(0.9,-52);
            telemetry.addData("angle", robot.realgyro.getIntegratedZValue());
            telemetry.update();
            sleep(200);
            succ();
            encoderMecanumDrive(0.75,105,4,-1,0.98);
            sleep(100);
            encoderMecanumDrive(1,90,4,0,-1);
            succstop();
            robot.clamper.setPosition(0.2);
            gyroTurn(1,-90);
            telemetry.addData("angle", robot.realgyro.getIntegratedZValue());
            telemetry.update();
            dumbencoderMecanumDrive(1,155, 4, 0,-1,true);
            robot.clamper.setPosition(0.03);
            dumbencoderMecanumDrive(1,30, 4, 0,1,false);
            gyroTurn(1,-100);
            dumbencoderMecanumDrive(1,60, 4, 0,1,false);




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
