package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Autonomous(name = "RedDepotAuto",group = "SkyStone")
public class RedDepotAuto extends BaseAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {

        //Initialize the hardware using BaseAutonomous Function
        inithardware(false);


        //Wait for the start button to be pressed
        waitForStart();
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        String location = vuforiaJoint(haddi,buddi);

        telemetry.addData("location", location);
        telemetry.update();




        if (location.equals("Center")){
            //move to block
            encoderMecanumDrive(0.9, 60,5,1,0);
            encoderMecanumDrive(0.55,18,1,0,-1);

            rightClamp();

            encoderMecanumDrive(0.65,25,5,1,0);

            bruhhh();
            sleep(500);
            liftClamp();
            encoderMecanumDrive(0.9,20,5,-1,0);
            encoderMecanumDrive(1.0,195,10,0,1);
            encoderMecanumDrive(0.9,31,5,1,0);
//            dropThaBlock();
            yeetThaBlock();
            gyroTurn(0.5,0,2);
            retractNoWait();
            encoderMecanumDrive(0.9,31,10,-1,0);
            bencoderMecanumDrive(1.0,225,10,180);
            encoderMecanumDrive(0.65,30,5,0,-1);
            rightClamp();

            encoderMecanumDrive(0.65,25,5,1,0);

            bruhhh();
            sleep(500);
            liftClamp();
            encoderMecanumDrive(0.9,28,5,-1,0);
            encoderMecanumDrive(1.0,270,10,0,1);
            encoderMecanumDrive(0.9,34,5,1,0);
//            dropThaBlock();
            yeetThaBlock();
            encoderMecanumDrive(0.65,15,5,-1,0);
            gyroTurn(0.7,90,2);
            encoderMecanumDrive(0.9,15,5,1,0);
            retractNoWait();
            grabWithPower(0.6);
            encoderMecanumDrive(0.9,20,5,0,-1);

            grab(false);
            encoderMecanumDrive(1,45,5,0,1);
//            gyroCurve(0.3,180,0,0.5);
            gyroCurve(1.0,180,0,0.6);
            gyroTurn(1.0,180,7);
            release();
            encoderMecanumDrive(1.0,15,5,0,-1);
//            encoderMecanumDrive(1.0,20,5,-1,0);
//            encoderMecanumDrive(1.0,70,5,0,1);
            spit();

            encoderMecanumDriveDirection(1,130,5,-162);



        } else if (location.equals("Right")){


            //move to block
            encoderMecanumDrive(0.9, 60,5,1,0);
            encoderMecanumDrive(0.5,60,1,0,-1);

            rightClamp();

            encoderMecanumDrive(0.5,25,5,1,0);

            bruhhh();
            sleep(500);
            liftClamp();

            encoderMecanumDrive(0.9,25,5,-1,0);
//            encoderMecanumDrive(0.9,215,10,0,1);
            encoderMecanumDriveDirection(0.9,215,8,0);

            encoderMecanumDrive(0.9,31,5,1,0);
//            dropThaBlock();
            yeetThaBlock();
            retractNoWait();
            gyroTurn(0.5,0,2);
//            robot.turnoright.setPosition(0.48);
            encoderMecanumDrive(0.9,28,10,-1,0);
//            robot.extendoright.setPosition(0);
//            gyroTurnAndMove(0.9,180,0.8,180);
            spit();
            sleep(200);
            succstop();
//            gyroTurn(0.5,180,2);
            encoderMecanumDriveDirection(0.9,72,5,180);
            encoderMecanumDriveDirection(0.9,108,6,-130);

            succ();
            encoderMecanumDriveDirection(0.5,30,3,180);





//            encoderMecanumDrive(0.9,20,5,-1,0);
//
//            encoderMecanumDrive(0.9,217.5,10,0,1);
//            encoderMecanumDrive(0.9,31,5,1,0);
//            dropThaBlock();
//            gyroTurn(0.5,0,2);
//            robot.turnoright.setPosition(0.48);
//            encoderMecanumDrive(0.9,31,10,-1,0);


        } else {

            //move to block
            encoderMecanumDrive(0.9, 60,5,1,0);


            rightClamp();

            encoderMecanumDrive(0.5,25,5,1,0);

            bruhhh();
            sleep(500);
            liftClamp();
            encoderMecanumDrive(0.9,20,5,-1,0);
            encoderMecanumDrive(0.9,172.5,10,0,1);
            encoderMecanumDrive(0.9,25,5,1,0);
//            dropThaBlock();
            yeetThaBlock();
            gyroTurn(0.5,0,2);

            encoderMecanumDrive(0.9,25,10,-1,0);
            robot.turnoright.setPosition(0.48);
            //Head back to pick up the second skystone
            bencoderMecanumDrive(0.9,233,10,180);
            rightClamp();

            encoderMecanumDrive(0.5,30,5,1,0);

            bruhhh();
            sleep(500);
            liftClamp();
            encoderMecanumDrive(0.9,28,5,-1,0);
            encoderMecanumDrive(0.9,248,10,0,1);
            encoderMecanumDrive(0.9,34,5,1,0);
//            dropThaBlock();
            yeetThaBlock();
            encoderMecanumDrive(0.5,15,5,-1,0);
            gyroTurn(0.7,90,2);
            encoderMecanumDrive(0.9,15,5,1,0);
            encoderMecanumDrive(0.9,30,5,0,-1);
            grabAndRetract();
            gyroCurve(1.0,180,0,0.6);
            gyroTurn(1.0,180,7);
            release();
            encoderMecanumDrive(1.0,15,5,0,-1);
            encoderMecanumDrive(1.0,20,5,-1,0);
            encoderMecanumDrive(0.9,65,5,0,1);
            spit();

        }





    }
}
