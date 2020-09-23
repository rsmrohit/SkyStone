package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Odometry Calibration", group="Odometry")
public class OdometryCalibration extends LinearOpMode {
    HardwareSkyStone robot = null;
    BNO055IMU imu;
    Drivetrain drivetrain;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        //Initialize IMU hardware map value. PLEASE UPDATE THIS VALUE TO MATCH YOUR CONFIGURATION
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        //Initialize IMU parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        telemetry.addData("Odometry System Calibration Status", "IMU Init Complete");
        telemetry.clear();

        waitForStart();

        while(getZAngle() < 90 && opModeIsActive()){
            drivetrain.forward(0.5);
            if(getZAngle() < 60) {
                drivetrain.forward(0.5);
            }else{
                drivetrain.forward(0.3);
            }
        }
        telemetry.addData("IMU Angle", getZAngle());
        telemetry.update();
    }

    private double getZAngle(){
        return (-imu.getAngularOrientation().firstAngle);
    }
}
