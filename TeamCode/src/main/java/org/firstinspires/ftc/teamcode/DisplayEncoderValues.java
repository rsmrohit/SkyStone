package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Display Encoder Values",group = "SkyStone")
public class DisplayEncoderValues extends BaseAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {
        inithardware(false);
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("vertical left", robot.verticalLeft.getCurrentPosition());
            telemetry.addData("vertical right", robot.verticalRight.getCurrentPosition());
            telemetry.addData("horizontal", robot.horizontal.getCurrentPosition());
            telemetry.update();
        }

    }

}
