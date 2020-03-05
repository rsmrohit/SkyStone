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
//        encoderMecanumDrive(0.9,35,5,0,1);
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        String location = vuforiaJoint(haddi,buddi);

        telemetry.addData("location", location);
        telemetry.update();




        if (location.equals("Center")){
            //move to block
            encoderMecanumDrive(0.9, 60,5,1,0);
            encoderMecanumDrive(0.4,22.8,1,0,-1);

            rightClamp();

            encoderMecanumDrive(0.5,25,5,1,0);

            bruhhh();
            sleep(500);
            liftClamp();
            encoderMecanumDrive(0.9,20,5,-1,0);
            encoderMecanumDrive(0.9,175,10,0,1);
            encoderMecanumDrive(0.9,31,5,1,0);
            dropThaBlock();
            gyroTurn(0.5,0);
            robot.turnoright.setPosition(0.48);
            encoderMecanumDrive(0.9,31,10,-1,0);
            gyroTurnAndMove(0.8,180,0.5,180);



        } else if (location.equals("Right")){


            //move to block
            encoderMecanumDrive(0.9, 60,5,1,0);
            encoderMecanumDrive(0.4,43.1,1,0,-1);

            rightClamp();

            encoderMecanumDrive(0.5,25,5,1,0);

            bruhhh();
            sleep(500);
            liftClamp();
            encoderMecanumDrive(0.9,20,5,-1,0);
            encoderMecanumDrive(0.9,175,10,0,1);
            encoderMecanumDrive(0.9,31,5,1,0);
            dropThaBlock();
            gyroTurn(0.5,0);
            robot.turnoright.setPosition(0.48);
            encoderMecanumDrive(0.9,31,10,-1,0);
            gyroTurnAndMove(0.8,180,0.5,180);


        } else {

            //move to block
            encoderMecanumDrive(0.9, 60,5,1,0);
            encoderMecanumDrive(0.4,2.5,1,0,-1);

            rightClamp();

            encoderMecanumDrive(0.5,25,5,1,0);

            bruhhh();
            sleep(500);
            liftClamp();
            encoderMecanumDrive(0.9,20,5,-1,0);
            encoderMecanumDrive(0.9,175,10,0,1);
            encoderMecanumDrive(0.9,31,5,1,0);
            dropThaBlock();
            gyroTurn(0.5,0);
            robot.turnoright.setPosition(0.48);
            encoderMecanumDrive(0.9,31,10,-1,0);

            //Head back to pick up the second skystone
            bencoderMecanumDrive(0.9,240,10,0,-1);
            rightClamp();

            encoderMecanumDrive(0.5,30,5,1,0);

            bruhhh();
            sleep(500);
            liftClamp();
            encoderMecanumDrive(0.9,25,5,-1,0);
            encoderMecanumDrive(0.9,255,10,0,1);
            encoderMecanumDrive(0.9,31,5,1,0);
            dropThaBlock();
            encoderMecanumDrive(0.5,15,5,-1,0);
            gyroTurn(0.7,90);
            encoderMecanumDrive(0.9,15,5,1,0);
            encoderMecanumDrive(0.9,30,5,0,-1);
            robot.turnoright.setPosition(0.48);
            grab();
            sleep(1000);
            encoderMecanumDrive(0.9,30,5,0,1);
            gyroCurve(0.7,180,0,0.5);
            release();
            sleep(500);
            encoderMecanumDrive(0.9,50,5,0,1);


        }





    }
}
