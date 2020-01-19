package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name = "Bruhtonomous",group = "SkyStone")
public class Testing extends BaseAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {

        //Initialize the hardware using BaseAutonomous Function
        inithardware(false);

        telemetry.addData("program","initing");
        telemetry.addData("initial position", robot.verticalSlider.getCurrentPosition());
        telemetry.update();


        //Wait for the start button to be pressed
        waitForStart();

        robot.verticalSlider.setTargetPosition(2000);
        robot.verticalSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.verticalSlider.setPower(0.5);

        while (robot.verticalSlider.isBusy()) {

            telemetry.addData("target position", robot.verticalSlider.getTargetPosition());

            telemetry.addData("current", robot.verticalSlider.getCurrentPosition());


            telemetry.update();
        }

        telemetry.addData("dumbass", " done");
        // Stop all motion;
        robot.verticalSlider.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.verticalSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

}
