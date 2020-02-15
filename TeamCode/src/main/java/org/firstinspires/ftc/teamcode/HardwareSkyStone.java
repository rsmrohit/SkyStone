
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
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
 * In this case that robot is a bruh.
 *
 */
public class HardwareSkyStone  {


    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backLeft = null;
    public DcMotor backRight = null;

    public DcMotor verticalSlider = null;
    public DcMotor horizontalSlider = null;

    public DcMotor verticalLeft = null;
    public DcMotor verticalRight = null;
    public DcMotor horizontal = null;


    public Servo leftclaw = null;
    public Servo rightclaw = null;

    public Servo clamper = null;

    public Servo capstone = null;

    public DcMotor  spinner = null;
    public DcMotor spinner2 = null;

    private boolean test;

    public BNO055IMU imu;

    public ModernRoboticsI2cGyro realgyro;

    public static final double wheelCircumfrence = (100*Math.PI/25.4);



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


        frontLeft = hwMap.get(DcMotor.class, "front_left");
        frontRight = hwMap.get(DcMotor.class, "front_right");
        backRight = hwMap.get(DcMotor.class, "back_right");
        backLeft = hwMap.get(DcMotor.class, "back_left");

        //Define odometry "motors"
        verticalLeft = hwMap.dcMotor.get("front_left");
        verticalRight = hwMap.dcMotor.get("front_right");
        horizontal = hwMap.dcMotor.get("back_right");

        realgyro = hwMap.get(ModernRoboticsI2cGyro.class, "gyro");
        realgyro.calibrate();


        if (!test) {
            verticalSlider = hwMap.get(DcMotor.class, "vert_slider");
            horizontalSlider = hwMap.get(DcMotor.class, "horz_slider");

            spinner = hwMap.get(DcMotor.class, "succ_1");
            spinner2 = hwMap.get(DcMotor.class,"succ_2");
            leftclaw = hwMap.get(Servo.class, "left_claw");
            rightclaw = hwMap.get(Servo.class, "right_claw");
            clamper = hwMap.get(Servo.class, "clamper");
            capstone = hwMap.get(Servo.class,"capstone_servo");

            verticalSlider.setDirection(DcMotor.Direction.REVERSE);
            spinner.setDirection(DcMotor.Direction.REVERSE);
            spinner2.setDirection(DcMotor.Direction.FORWARD);

            spinner.setPower(0);
            spinner2.setPower(0);
            verticalSlider.setPower(0);
            horizontalSlider.setPower(0);

            rightclaw.setPosition(0.5);
            leftclaw.setPosition(0.5);
            clamper.setPosition(0.03);
            capstone.setPosition(0.2);

        }


        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);


        realgyro.resetZAxisIntegrator();
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


    }
    public void setMode(String mode){
        if (mode.equals(TeleOpRunMode)){

            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            if (!test){
                spinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                spinner2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                verticalSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                horizontalSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            }

        }
        else {

            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            if (!test){
                spinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                spinner2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                verticalSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                verticalSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                horizontalSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                horizontalSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

        }
    }
    public void updateDriveTrainInputs(MecanumWheels wheels){
        frontRight.setPower(wheels.getFrontRightPower());
        frontLeft.setPower(wheels.getFrontLeftPower());
        backLeft.setPower(wheels.getRearLeftPower());
        backRight.setPower(wheels.getRearRightPower());
    }
}
