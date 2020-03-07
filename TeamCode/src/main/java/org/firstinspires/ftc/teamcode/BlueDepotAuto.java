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
        telemetry.addData("imu", robot.imu.getAngularOrientation().firstAngle);
        telemetry.update();

        //Wait for the start button to be pressed
        waitForStart();
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        String location = vuforiaJointo(haddi,buddi);

        telemetry.addData("location", location);
        telemetry.update();
        sleep(1000);

        if (location.equals("Center")){
            //move to block
            encoderMecanumDriveDirection(0.9, 75,5,-108);


            rightClamp();

            encoderMecanumDrive(0.5,28,5,1,0);

            bruhhh();
            sleep(500);
            liftClamp();
            telemetry.addData("imu value",robot.imu.getAngularOrientation().firstAngle);
            telemetry.update();

            gyroTurn(0.5,0,1);
            telemetry.addData("imu value",robot.imu.getAngularOrientation().firstAngle);
            telemetry.update();

            encoderMecanumDriveDirection(0.9,23,5,90);


            encoderMecanumDriveDirection(1.0,177,10,180);
            encoderMecanumDrive(0.9,27,5,1,0);
            yeetThaBlock();
            gyroTurn(0.5,0,1);

            encoderMecanumDriveDirection(0.9,25,3,90);
            robot.turnoright.setPosition(0.48);
            robot.extendoright.setPosition(0);

            //Head back to pick up the second skystone
            bencoderMecanumDrive(0.9,240,10,0);
            rightClamp();

            encoderMecanumDriveDirection(0.5,25,5,-90);

            bruhhh();
            sleep(500);
            liftClamp();

            encoderMecanumDriveDirection(0.9,28,5,90);

            encoderMecanumDriveDirection(1.0,275,10,180);
            encoderMecanumDrive(0.9,35,5,1,0);
            yeetThaBlock();
            encoderMecanumDriveDirection(0.5,15,5,90);
            gyroTurn(0.7,90,2);

            encoderMecanumDriveDirection(0.9,16,5,-90);
            robot.turnoright.setPosition(0.48);
            robot.extendoright.setPosition(0);
            sleep(1000);

            grab();
            encoderMecanumDriveDirection(1,30,5,90);
            gyroCurve(1.0,0,0,0.6);
            gyroTurn(1.0,0,7);
            release();
            encoderMecanumDriveDirection(1.0,15,5,180);
//            encoderMecanumDrive(1.0,25,5,1,0);
//            encoderMecanumDrive(1.0,91,5,0,1);
//            spit();


        }else if(location.equals("Right")){



        }else{
            //move to block
            encoderMecanumDrive(0.9, 60,5,1,0);


            rightClamp();

            encoderMecanumDrive(0.5,25,5,1,0);

            bruhhh();
            sleep(500);
            liftClamp();
            telemetry.addData("imu value",robot.imu.getAngularOrientation().firstAngle);
            telemetry.update();

            gyroTurn(0.5,0,1);
            telemetry.addData("imu value",robot.imu.getAngularOrientation().firstAngle);
            telemetry.update();

            encoderMecanumDrive(0.9,23,5,-1,0);

            sleep(1000);
            encoderMecanumDrive(1.0,201.5,10,0,-1);
            encoderMecanumDrive(0.9,26,5,1,0);
            yeetThaBlock();
            gyroTurn(0.5,0,1);

            encoderMecanumDrive(0.9,27,10,-1,0);
            robot.turnoright.setPosition(0.48);
            robot.extendoright.setPosition(0);
            //Head back to pick up the second skystone
            bencoderMecanumDrive(0.9,259,10,0);
            encoderMecanumDrive(0.3,7,0.4,0,1);
            rightClamp();

            encoderMecanumDrive(0.5,28,5,1,0);

            bruhhh();
            sleep(500);
            liftClamp();
            encoderMecanumDrive(0.9,29,5,-1,0);

            encoderMecanumDrive(1.0,292,10,0,-1);
            encoderMecanumDrive(0.9,27,5,1,0);
            yeetThaBlock();
            encoderMecanumDrive(0.5,15,5,-1,0);
            gyroTurn(0.7,90,2);

            encoderMecanumDrive(0.9,19,5,0,-1);
            robot.turnoright.setPosition(0.48);
            robot.extendoright.setPosition(0);
            sleep(1000);

            grab();
            encoderMecanumDrive(1,30,5,0,1);
            gyroCurve(0.5,0,0,0.6);
            gyroTurn(1.0,0,7);
            release();
            encoderMecanumDrive(1.0,15,5,0,-1);
//            encoderMecanumDrive(1.0,25,5,1,0);
//            encoderMecanumDrive(1.0,91,5,0,1);
//            spit();

        }


    }
}
