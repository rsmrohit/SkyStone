package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.HardwareSkyStone.TeleOpRunMode;

@TeleOp(name="TestMotor", group="RoverRuckus")
public class TestTeleop extends OpMode {

    HardwareSkyStone robot       = new HardwareSkyStone(false);


    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.setMode(TeleOpRunMode);
    }

    @Override
    public void loop() {
        if (gamepad1.a){
            robot.extendoright.setPosition(1);
            robot.turnoright.setPosition(0);

        }else if(gamepad1.b){
            robot.extendoright.setPosition(0.5);
            robot.turnoright.setPosition(0.18);
        }else if (gamepad1.x){
            robot.clamper.setPosition(0.13);
        }
        telemetry.addData("imu value", robot.imu.getAngularOrientation());
        telemetry.update();
    }
}
