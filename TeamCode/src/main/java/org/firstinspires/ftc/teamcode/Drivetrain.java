package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Drivetrain {
    HardwareSkyStone robot;

    public void setPower(motorPower... mp){
        for (motorPower current : mp) {
            current.motor.setPower(current.power);
        }
    }

    public void forward(double power){
        setPower(new motorPower(robot.backLeft, power), new motorPower(robot.backRight, power), new motorPower(robot.frontLeft, power), new motorPower(robot.frontRight, power));
    }
}
