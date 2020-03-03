package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

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
//        double[] followAngles = {Math.toRadians(90),Math.toRadians(90),Math.toRadians(90), Math.toRadians(90), Math.toRadians(90)};


        inithardware(false);


//        RobotMovement r = new RobotMovement(updateBoi,robot,telemetry);
//        ArrayList<CurvePoint> allPoints = new ArrayList<>();
//        allPoints.add(new CurvePoint(0*COUNTS_PER_INCH , -COUNTS_PER_INCH,0.4 ,0 ,10*COUNTS_PER_INCH, Math.toRadians(30), 0.5));
//        allPoints.add(new CurvePoint(0*COUNTS_PER_INCH , 30*COUNTS_PER_INCH,0.4 ,0.2 ,10*COUNTS_PER_INCH, Math.toRadians(30), 0.5));
//        allPoints.add(new CurvePoint(30*COUNTS_PER_INCH , 30*COUNTS_PER_INCH,0.4,0.2 ,9*COUNTS_PER_INCH, Math.toRadians(30), 0.5));
//        allPoints.add(new CurvePoint(30*COUNTS_PER_INCH , 0*COUNTS_PER_INCH,0.4,0.2 ,5*COUNTS_PER_INCH, Math.toRadians(30), 0.5));
//        allPoints.add(new CurvePoint(0*COUNTS_PER_INCH , 0*COUNTS_PER_INCH,0.4,0.2 ,5*COUNTS_PER_INCH, Math.toRadians(30), 0.5));



        waitForStart();
//        encoderMecanumDrive(0.6,50,3,0,-1);
//        Thread.sleep(1000);
//        encoderMecanumDrive(0.6,50,3,0,1);
//        Thread.sleep(1000);
//        encoderMecanumDrive(0.6,50,3,0,-1);
//        Thread.sleep(1000);
//        encoderMecanumDrive(0.6,50,3,0,1);
//        gyroCurve(0.15,90,0,0.3);
        gyroTurn(0.7,-100);




//        while (!isStopRequested()){
//            updateBoi.globalCoordinatePositionUpdate();
//            r.followCurve(allPoints, followAngles);

//        }

    }

}
