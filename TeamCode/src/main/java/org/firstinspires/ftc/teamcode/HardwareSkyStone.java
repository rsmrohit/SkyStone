
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a RoverRuckus.
 *
 */
public class HardwareSkyStone  {

    /* Public OpMode members. */
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backLeft = null;
    public DcMotor backRight = null;


    public DcMotor  raiser = null;

    public DcMotor hanger = null;

    public CRServo leftclaw = null;
    public CRServo rightclaw = null;

    public DcMotor slider = null;
    public DcMotor  spinner = null;
    public DcMotor spinner2 = null;

    private boolean test = true;





    public ModernRoboticsI2cGyro realgyro;
    public ModernRoboticsI2cGyro realgyro2;


    public static final String TeleOpRunMode = "no encoders";
    public static final double ARM_UP_POSITION = 0.7;


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




        if (!test) {
            //Define and Initialize Sensors
            realgyro = hwMap.get(ModernRoboticsI2cGyro.class, "gyro");
            realgyro.calibrate();


            realgyro2 = hwMap.get(ModernRoboticsI2cGyro.class, "gyro2");
            realgyro2.calibrate();

            //Liner Slide extend/contract motors

            slider = hwMap.get(DcMotor.class, "slider");


            // Sliders arm lowering/raising motoros
            raiser = hwMap.get(DcMotor.class, "raiser");
            hanger = hwMap.get(DcMotor.class, "hanger");


            //bucket dumping motor




        }


        frontLeft.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        frontRight.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        spinner.setDirection(DcMotor.Direction.REVERSE);
        spinner2.setDirection(DcMotor.Direction.FORWARD);

        if (!test) {
            raiser.setDirection(DcMotor.Direction.REVERSE);
            hanger.setDirection(DcMotor.Direction.REVERSE);

            slider.setDirection(DcMotor.Direction.REVERSE);

        }



        // Set all motors to zero power
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
        spinner.setPower(0);
        spinner2.setPower(0);

        leftclaw.setPower(0);
        rightclaw.setPower(0);


        if (!test) {
            realgyro.resetZAxisIntegrator();
            raiser.setPower(0);
            hanger.setPower(0);

            slider.setPower(0);


        }

    }
    public void setMode(String mode){
        if (mode.equals(TeleOpRunMode)){

            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            spinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            spinner2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            if (!test) {
                raiser.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                hanger.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            }
        }
        else {

            // Reset encoder
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            spinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            spinner2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            if (!test) {
                raiser.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                hanger.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            }


            // set to run using encoder
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            if (!test) {
                raiser.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                hanger.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            }


        }
    }
}
