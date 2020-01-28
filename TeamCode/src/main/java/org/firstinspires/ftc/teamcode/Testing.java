package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name = "Bruhtonomous",group = "SkyStone")
public class Testing extends BaseAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {

        //Initialize the hardware using BaseAutonomous Function
        inithardware(false);
        telemetry.addData("at position", robot.clamper.getPosition());

        //Wait for the start button to be pressed
        waitForStart();

//        for (double i =0;i< 1;i = i + 0.1){
//
//            robot.clamper.setPosition(i);
//            telemetry.addData("at position", robot.clamper.getPosition());
//            telemetry.update();
//            sleep(1000);
//
//        }
//        robot.horizontalSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.horizontalSlider.setPower(0.8);
//        sleep(1000);

        dumbencoderMecanumDrive(1,130, 4, 0,1,false);

    }

}
