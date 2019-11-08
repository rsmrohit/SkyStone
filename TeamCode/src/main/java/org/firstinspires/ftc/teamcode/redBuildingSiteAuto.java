package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "redBuildingSiteAuto",group = "SkyStone")
public class redBuildingSiteAuto extends BaseAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {
        inithardware(false);
        waitForStart();
        encoderMecanumDrive(DRIVE_SPEED,68,2,0,-1);
        encoderMecanumDrive(DRIVE_SPEED,27,2,-1,0);
        encoderMecanumDrive(0.4,6.5,2,0,-1);
        grab();
        robot.rightclaw.setPower(-0.1);
        robot.leftclaw.setPower(-0.1);
        encoderMecanumDrive(DRIVE_SPEED,114,2,0,1);
        robot.rightclaw.setPower(0);
        robot.leftclaw.setPower(0);
        release();
        encoderMecanumDrive(DRIVE_SPEED,7,2,0,-1);
        encoderMecanumDrive(DRIVE_SPEED,90,2,1,0);
        encoderMecanumDrive(DRIVE_SPEED,20,2,0,-1);
        encoderMecanumDrive(DRIVE_SPEED,70,2,-1,0);
        encoderMecanumDrive(DRIVE_SPEED,20,2,1,0);
        correctGyroTurn(0.6,193);
        encoderMecanumDrive(DRIVE_SPEED,95,2,-1,0);
        correctGyroTurn(0.6,155);
        encoderMecanumDrive(DRIVE_SPEED,60,2,0,1);








    }

}