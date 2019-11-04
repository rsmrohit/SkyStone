package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "redDepot",group = "SkyStone")
public class moveToVuforia extends BaseAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {

        //Initialize the hardware using BaseAutonomous Function
        inithardware(false);

        telemetry.addData("program","initialized");
        telemetry.update();


        //Wait for the start button to be pressed
        waitForStart();

        encoderMecanumDrive(0.5,40,2,0,1);
        encoderMecanumDrive(0.5,50,2,1,0);
        String location = vuforiaJoint(haddi,buddi);
        telemetry.addData("location",location);
        if (location.equals("Center")){
            encoderMecanumDrive(0.4,5,2,1,0);
            robot.spinner.setPower(-1.0 );
            robot.spinner2.setPower(-1.0 );
            encoderMecanumDrive(0.4,100,6,0,1);
        }else if (location.equals("Right")){
            encoderMecanumDrive(0.4,5,2,1,0);
            gyroTurn(0.4,-45);
            robot.spinner.setPower(-1.0 );
            robot.spinner2.setPower(-1.0 );
            encoderMecanumDrive(0.4,100,6,-1,1);

        }else if (location.equals("Left")){
            gyroTurn(0.4,45);
            robot.spinner.setPower(-1.0 );
            robot.spinner2.setPower(-1.0 );
            encoderMecanumDrive(0.4,100,6,1,1);
        }



    }
}
