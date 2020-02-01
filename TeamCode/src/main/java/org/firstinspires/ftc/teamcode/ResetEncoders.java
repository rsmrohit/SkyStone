package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "ResetEncoders",group = "SkyStone")
public class ResetEncoders extends BaseAutonomous {
    @Override
    public void runOpMode() throws InterruptedException {
        inithardware(false);
        waitForStart();
        telemetry.addData("horizontal",robot.horizontalSlider.getCurrentPosition());


    }

}
