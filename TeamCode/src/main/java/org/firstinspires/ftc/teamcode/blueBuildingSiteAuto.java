package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Autonomous(name = "blueBuildingSiteAuto",group = "SkyStone")
public class blueBuildingSiteAuto extends BaseAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {
        inithardware(false);
        waitForStart();
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        encoderMecanumDrive(DRIVE_SPEED,65,2,0,-1);
        encoderMecanumDrive(DRIVE_SPEED,30,2,1,0);
        encoderMecanumDrive(0.5,22.5,2,0,-1);
        sleep(1000);
        grab();
        sleep(1000);

        encoderMecanumDrive(1.0,50,5,0,1);
        gyroCurve(0.5,90,0,1);
        release();
        encoderMecanumDrive(0.6,50,5,0,-1);

        //go sideways into the wall
//        encoderMecanumDrive(1,5,1,0,1);
        gyroTurn(0.7,90,2);
        encoderMecanumDrive(1.0,50,2,1,0);

        encoderMecanumDrive(1.0,80,4,0,1);





    }

}
