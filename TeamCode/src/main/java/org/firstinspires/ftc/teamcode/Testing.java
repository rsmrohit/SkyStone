package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.PurePursuit.CurvePoint;

import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.PurePursuit.RobotMovement;

import java.util.ArrayList;


@Autonomous(name = "Bruhtonomous",group = "SkyStone")
public class Testing extends BaseAutonomous {
    RobotMovement r;
    public static double[] followAngles = {Math.toRadians(0),Math.toRadians(0),Math.toRadians(0), Math.toRadians(-180), Math.toRadians(-180), Math.toRadians(-90), Math.toRadians(0),
            Math.toRadians(0), Math.toRadians(0), Math.toRadians(-180),Math.toRadians(-180),Math.toRadians(0)};
    final double COUNTS_PER_INCH = (383.6*2/robot.wheelCircumfrence);



    @Override
    public void runOpMode() throws InterruptedException {
        inithardware(false);
        OdometryGlobalCoordinatePosition g = new OdometryGlobalCoordinatePosition(robot.verticalLeft, robot.verticalRight, robot.horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(g);
        positionThread.start();
        r = new RobotMovement(g,robot);
        ArrayList<CurvePoint> allPoints = new ArrayList<>();
        allPoints.add(new CurvePoint(0 , 0,0.6 ,0.7 ,35, Math.toRadians(30), 0.5));
        allPoints.add(new CurvePoint(100 , 100,0.6 ,0.7 ,35, Math.toRadians(30), 0.5));
        allPoints.add(new CurvePoint(150 , 80,0.6,0.7 ,15, Math.toRadians(30), 0.5));



        //Uncomment these lines after you've tested out the lines above and they work like the simulation
        /*
        allPoints.add(new CurvePoint(50 , 180,0.6 ,0.5 ,35, Math.toRadians(30), 0.5));
        allPoints.add(new CurvePoint(50 , 230,0.6 ,0.5 ,35, Math.toRadians(30), 0.2));
        allPoints.add(new CurvePoint(100 , 280,0.6 ,0.5 ,25, Math.toRadians(20), 0.5));
        allPoints.add(new CurvePoint(50 , 230,0.6,0.5 ,35, Math.toRadians(30), 0.5));
        allPoints.add(new CurvePoint(50 , 80,0.6,0.5 ,35, Math.toRadians(30), 0.5));
        allPoints.add(new CurvePoint(150 , 50,0.6,0.5 ,25, Math.toRadians(30), 0.5));
        allPoints.add(new CurvePoint(80 , 150,0.6,0.7 ,35, Math.toRadians(30), 0.5));
        allPoints.add(new CurvePoint(80 , 300,0.9,1.0 ,35, Math.toRadians(30), 0.5));
        allPoints.add(new CurvePoint(80 , 120,0.9 ,1.0 ,35, Math.toRadians(30), 0.5));
*/

        //Initialize the hardware using BaseAutonomous Function

        inithardware(false);
        waitForStart();

        while (opModeIsActive()){
            r.followCurve(allPoints, followAngles);
        }


    }

}
