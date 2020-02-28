package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Odometry.UpdateBoi;
import org.firstinspires.ftc.teamcode.PurePursuit.CurvePoint;

import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.PurePursuit.RobotMovement;

import java.util.ArrayList;


@Autonomous(name = "Bruhtonomous",group = "SkyStone")
public class Testing extends BaseAutonomous {


    @Override
    public void runOpMode() throws InterruptedException {
        double[] followAngles = {Math.toRadians(0),Math.toRadians(0),Math.toRadians(0), Math.toRadians(-180), Math.toRadians(-180), Math.toRadians(-90), Math.toRadians(0),
                Math.toRadians(0), Math.toRadians(0), Math.toRadians(-180),Math.toRadians(-180),Math.toRadians(0)};
        final double COUNTS_PER_INCH = 307.699557;

        inithardware(false);
        UpdateBoi u = new UpdateBoi(robot.verticalLeft, robot.verticalRight, robot.horizontal, COUNTS_PER_INCH, 75);

        RobotMovement r = new RobotMovement(u,robot);
        ArrayList<CurvePoint> allPoints = new ArrayList<>();
        allPoints.add(new CurvePoint(0 , 0,0.3 ,0.3 ,35, Math.toRadians(30), 0.5));
        allPoints.add(new CurvePoint(0 , 20,0.3 ,0.3 ,35, Math.toRadians(30), 0.5));
        allPoints.add(new CurvePoint(10 , 40,0.3,0.3 ,15, Math.toRadians(30), 0.5));



        //Initialize the hardware using BaseAutonomous Function

        inithardware(false);
        waitForStart();

        while (opModeIsActive()){
            u.globalCoordinatePositionUpdate();
            r.followCurve(allPoints, followAngles);
            telemetry.addData("nextPointnum", r.nextPointNum);
            telemetry.addData("movex",r.movex);
            telemetry.addData("movey", r.movey);
            telemetry.addData("moveturn", r.moveturn);
            telemetry.update();
        }


    }

}
