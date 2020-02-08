package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.HardwareSkyStone;


import java.io.File;

/**
 * Created by Deez Nuts on 2/6/2020.
 * Odometry system calibration. Run this OpMode to generate the necessary constants to calculate the robot's global position on the field.
 * The Global Positioning Algorithm will not function and will throw an error if this program is not run first
 */
@TeleOp(name = "Odometry System Calibration", group = "Skystone")
public class OdometryCalibration extends LinearOpMode {

    HardwareSkyStone roboto = null;

    final double PIVOT_SPEED = 0.5;

    //The amount of encoder ticks for each inch the robot moves. THIS WILL CHANGE FOR EACH ROBOT AND NEEDS TO BE UPDATED HERE
    final double COUNTS_PER_INCH = 307.699557;

    ElapsedTime timer = new ElapsedTime();

    double horizontalTickOffset = 0;

    //Text files to write the values to. The files are stored in the robot controller under Internal Storage\FIRST\settings
    File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    @Override
    public void runOpMode() throws InterruptedException {
        roboto = new HardwareSkyStone(true);
        roboto.init(hardwareMap);
        roboto.setMode("encoders lmao");


        //Odometry System Calibration Init Complete
        telemetry.addData("Odometry System Calibration Status", "Init Complete");
        telemetry.update();

        waitForStart();

        //Begin calibration (if robot is unable to pivot at these speeds, please adjust the constant at the top of the code
        while(roboto.imu.getAngularOrientation().firstAngle > -90 && opModeIsActive()){
            roboto.frontRight.setPower(-PIVOT_SPEED);
            roboto.backRight.setPower(-PIVOT_SPEED);
            roboto.frontLeft.setPower(PIVOT_SPEED);
            roboto.backLeft.setPower(PIVOT_SPEED);

            if(roboto.imu.getAngularOrientation().firstAngle > -60) {
                setPowerAll(-PIVOT_SPEED, -PIVOT_SPEED, PIVOT_SPEED, PIVOT_SPEED);
            }else{
                setPowerAll(-PIVOT_SPEED/2, -PIVOT_SPEED/2, PIVOT_SPEED/2, PIVOT_SPEED/2);
            }

            telemetry.addData("Angle", roboto.imu.getAngularOrientation().firstAngle);
            telemetry.update();
        }

        //Stop the robot
        setPowerAll(0, 0, 0, 0);
        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){
            telemetry.addData("Angle", roboto.imu.getAngularOrientation().firstAngle);
            telemetry.update();
        }

        //Record IMU and encoder values to calculate the constants for the global position algorithm
        double angle = roboto.imu.getAngularOrientation().firstAngle;

        /*
        Encoder Difference is calculated by the formula (leftEncoder - rightEncoder)
        Since the left encoder is also mapped to a drive motor, the encoder value needs to be reversed with the negative sign in front
        THIS MAY NEED TO BE CHANGED FOR EACH ROBOT
       */
        double encoderDifference = Math.abs(roboto.verticalLeft.getCurrentPosition()) + (Math.abs(roboto.verticalRight.getCurrentPosition()));

        double verticalEncoderTickOffsetPerDegree = encoderDifference/angle;

        double wheelBaseSeparation = (2*90*verticalEncoderTickOffsetPerDegree)/(Math.PI*COUNTS_PER_INCH);

        horizontalTickOffset = roboto.horizontal.getCurrentPosition()/Math.toRadians(roboto.imu.getAngularOrientation().firstAngle);

        //Write the constants to text files
        ReadWriteFile.writeFile(wheelBaseSeparationFile, String.valueOf(wheelBaseSeparation));
        ReadWriteFile.writeFile(horizontalTickOffsetFile, String.valueOf(horizontalTickOffset));

        while(opModeIsActive()){
            telemetry.addData("Odometry System Calibration Status", "Calibration Complete");
            //Display calculated constants
            telemetry.addData("Wheel Base Separation", wheelBaseSeparation);
            telemetry.addData("Horizontal Encoder Offset", horizontalTickOffset);

            //Display raw values
            telemetry.addData("IMU Angle", roboto.imu.getAngularOrientation().firstAngle);
            telemetry.addData("Vertical Left Position", roboto.verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical Right Position", roboto.verticalRight.getCurrentPosition());
            telemetry.addData("Horizontal Position", roboto.horizontal.getCurrentPosition());
            telemetry.addData("Vertical Encoder Offset", verticalEncoderTickOffsetPerDegree);

            telemetry.update();
        }
    }


    /**
     * Sets power to all four drive motors
     * @param rf power for right front motor
     * @param rb power for right back motor
     * @param lf power for left front motor
     * @param lb power for left back motor
     */
    private void setPowerAll(double rf, double rb, double lf, double lb){
        roboto.frontRight.setPower(rf);
        roboto.backRight.setPower(rb);
        roboto.frontLeft.setPower(lf);
        roboto.backLeft.setPower(lb);
    }

}
