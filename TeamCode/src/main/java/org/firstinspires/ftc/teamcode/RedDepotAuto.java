package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RedDepotAuto",group = "SkyStone")
public class RedDepotAuto extends BaseAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {

        //Initialize the hardware using BaseAutonomous Function
        inithardware(false);

        telemetry.addData("program","initialized");
        telemetry.addData("horizontal encoder", robot.horizontalSlider.getCurrentPosition());
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
            //first block
            //strafe and move forward for block setup
            encoderMecanumDrive(0.75,32.5,1,-1,0);
            encoderMecanumDrive(0.9,60,5,0,1);
            gyroTurn(0.9,-52);
            succ();
            //move to collect block
            encoderMecanumDrive(0.75,75,4,-1,0.95);
            //fine tune grab block
            encoderMecanumDrive(1,22.5,4,0.819152,-0.573576);
            //move back to be able to move to foundation
            encoderMecanumDrive(1,68,4,0,-1);
            succstop();
            robot.clamper.setPosition(0.2);
            //turn towards foundation
            gyroTurn(0.9,-85);
            telemetry.addData("angle", robot.realgyro.getIntegratedZValue());
            telemetry.update();
            encoderMecanumDrive(1,90,10,0,-1);
            //move to foundation
            bruh();
            robot.clamper.setPosition(0.03);

            //second block
            //coming back to get second block
            dumbencoderMecanumDrive(1,180, 4, 0,1,false);
            encoderMecanumDrive(1,55,10,-1,0);
            succ();
            encoderMecanumDrive(0.7,30,10,0,1);
            succstop();
            robot.clamper.setPosition(0.15);
            encoderMecanumDrive(1,130,10,1,-1);
            encoderMecanumDrive(1,130,10,0,-1);
            bruh();
            robot.clamper.setPosition(0.03);
            //move to park
            dumbencoderMecanumDrive(1,70, 4, 0,1,false);


            //turn to collect
//            gyroTurn(0.9,-48);
//            telemetry.addData("angle", robot.realgyro.getIntegratedZValue());
//            telemetry.update();
//            sleep(200);
            //start sucking

            /*

            //strafe to block pt 1
            encoderMecanumDrive(0.75,77,4,-0.93,1);
            //drive straight to separate blocks for collection
            encoderMecanumDrive(0.9,30,4,0,1);
            succstop();
            //move back to original position
            encoderMecanumDrive(1,80,4,0,-1);
            robot.clamper.setPosition(0.2);
            //turn to straight position
            gyroTurn(1,-85);
            telemetry.addData("angle", robot.realgyro.getIntegratedZValue());
            telemetry.update();
            //move to foundation
            encoderMecanumDrive(1,150, 4, 0,-1);
            bruh();
            robot.clamper.setPosition(0.03);
            //move to park
            dumbencoderMecanumDrive(1,70, 4, 0,1,false);


*/

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
