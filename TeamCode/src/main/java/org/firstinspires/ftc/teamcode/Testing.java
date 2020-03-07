package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Odometry.UpdateBoi;
import org.firstinspires.ftc.teamcode.PurePursuit.CurvePoint;

import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.PurePursuit.MathFunctions;
import org.firstinspires.ftc.teamcode.PurePursuit.RobotMovement;

import java.util.ArrayList;


@Autonomous(name = "Bruhtonomous",group = "SkyStone")
public class Testing extends BaseAutonomous {


    @Override
    public void runOpMode() throws InterruptedException {

        inithardware(false);
        double initiala  = robot.imu.getAngularOrientation().firstAngle;

        //Wait for the start button to be pressed
        waitForStart();
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

//        encoderMecanumDriveDirection(0.9, 72,5,-108);
        robot.tape.setPower(0.5);
        sleep(4000);

//        grab();
//        gyroCurve(0.3,initiala+90,0,0.5);
//        gyroTurn(1.0,initiala+90,7);
//        release();
//        encoderMecanumDrive(1.0,15,5,0,-1);
//        encoderMecanumDrive(1.0,20,5,-1,0);
//        encoderMecanumDrive(0.9,65,5,0,1);



    }

}
