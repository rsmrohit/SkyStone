package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class motorPower {
    public DcMotor motor;
    public double power;

    public motorPower(DcMotor motor, double power){
        this.motor = motor;
        this.power = power;
    }

}
