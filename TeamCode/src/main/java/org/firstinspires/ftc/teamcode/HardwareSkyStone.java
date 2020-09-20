
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.swing.internal.plaf.synth.resources.synth_sv;

import java.util.ArrayList;
import java.util.List;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a RoverRuckus.
 *
 */


public class HardwareSkyStone  {

    public DcMotor frontLeft, frontRight, backLeft, backRight, left, right, middle;

    public DcMotor  spinner = null;
    public DcMotor spinner2 = null;

    enum motornames{
        frontLeft, frontRight, backLeft, backRight, spinner, spinner2;
    }
    enum odowheelnames{
        left, middle, right
    }

    /* Public OpMode members. */
    public DcMotor getOdoMotor(odowheelnames wheel){
        switch (wheel){
            case left:
                return left;
            case right:
                return right;
            case middle:
                return middle;
        }
        return left;
    }


    public DcMotor getMotor(motornames motor){
       switch (motor){
           case frontLeft:
               return frontLeft;
           case backLeft:
               return backLeft;
           case backRight:
               return backRight;
           case frontRight:
               return frontRight;
           case spinner:
               return spinner;
           case spinner2:
               return spinner2;
       }
        return frontRight;
    }

    public CRServo leftclaw = null;
    public CRServo rightclaw = null;

    private boolean test;

    public ModernRoboticsI2cGyro realgyro;
    public ModernRoboticsI2cGyro realgyro2;


    public static final String TeleOpRunMode = "no encoders";



    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  =  new ElapsedTime();

    /* Constructor */
    public HardwareSkyStone(boolean test){
        this.test = test;
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors

        //Drive Train motors
        frontLeft = hwMap.get(DcMotor.class, "front_left");
        frontRight = hwMap.get(DcMotor.class, "front_right");
        backRight = hwMap.get(DcMotor.class, "back_right");
        backLeft = hwMap.get(DcMotor.class, "back_left");
        spinner = hwMap.get(DcMotor.class, "succ_1");
        spinner2 = hwMap.get(DcMotor.class,"succ_2");
        leftclaw = hwMap.get(CRServo.class, "left_claw");
        rightclaw = hwMap.get(CRServo.class, "right_claw");

        left = hwMap.dcMotor.get("leftVertical");
        right = hwMap.dcMotor.get("rightVertical");
        middle = hwMap.dcMotor.get("middleHorizontal");


        if (!test) {
            //Define and Initialize Sensors
            realgyro = hwMap.get(ModernRoboticsI2cGyro.class, "gyro");
            realgyro.calibrate();
//            realgyro2 = hwMap.get(ModernRoboticsI2cGyro.class, "gyro2");
//            realgyro2.calibrate();

        }


//        frontLeft.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
//        frontRight.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
//        backRight.setDirection(DcMotor.Direction.FORWARD);
//        backLeft.setDirection(DcMotor.Direction.REVERSE);
//        spinner.setDirection(DcMotor.Direction.REVERSE);
//        spinner2.setDirection(DcMotor.Direction.FORWARD);
        for (motornames motors : motornames.values()){
            if (motors.equals(motornames.frontLeft)|| motors.equals(motornames.spinner)|| motors.equals(motornames.backLeft)) {
                getMotor(motors).setDirection(DcMotor.Direction.REVERSE);
            } else {
                getMotor(motors).setDirection(DcMotor.Direction.FORWARD);
            }
        }
        rightclaw.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        if (!test) {
//
//
//        }


        // Set all motors to zero power
        for (motornames motors : motornames.values()){
            getMotor(motors).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            getMotor(motors).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            getMotor(motors).setPower(0);
        }
        for (odowheelnames motors : odowheelnames.values()){
            getOdoMotor(motors).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            getOdoMotor(motors).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        leftclaw.setPower(0);
        rightclaw.setPower(0);


        if (!test) {
            realgyro.resetZAxisIntegrator();

        }

    }
    public void setMode(String mode){
        if (mode.equals(TeleOpRunMode)){

            for (motornames motors : motornames.values()){
                getMotor(motors).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            if (!test) {

            }
        }
        else {

            // Reset encoder
            for (motornames motors : motornames.values()){
                if (motors.equals(motornames.spinner)|| motors.equals(motornames.spinner2)) {
                    getMotor(motors).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                } else {
                    getMotor(motors).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
            }

            if (!test) {


            }


            // set to run using encoder
            for (motornames motors : motornames.values()){
                if (motors.equals(motornames.spinner)|| motors.equals(motornames.spinner2)) {
                } else {
                    getMotor(motors).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
            }
//            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            if (!test) {


            }

        }
    }
}
