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
        double[] followAngles = {Math.toRadians(90),Math.toRadians(90),Math.toRadians(90), Math.toRadians(90), Math.toRadians(90)};
        final double COUNTS_PER_INCH = 307.699557;

        inithardware(false);
        UpdateBoi u = new UpdateBoi(robot.verticalLeft, robot.verticalRight, robot.horizontal, COUNTS_PER_INCH, robot.imu);

        RobotMovement r = new RobotMovement(u,robot,telemetry);
        ArrayList<CurvePoint> allPoints = new ArrayList<>();
        allPoints.add(new CurvePoint(0*COUNTS_PER_INCH , -COUNTS_PER_INCH,0.4 ,0 ,10*COUNTS_PER_INCH, Math.toRadians(30), 0.5));
        allPoints.add(new CurvePoint(0*COUNTS_PER_INCH , 30*COUNTS_PER_INCH,0.4 ,0.2 ,10*COUNTS_PER_INCH, Math.toRadians(30), 0.5));
        allPoints.add(new CurvePoint(30*COUNTS_PER_INCH , 30*COUNTS_PER_INCH,0.4,0.2 ,9*COUNTS_PER_INCH, Math.toRadians(30), 0.5));
        allPoints.add(new CurvePoint(30*COUNTS_PER_INCH , 0*COUNTS_PER_INCH,0.4,0.2 ,5*COUNTS_PER_INCH, Math.toRadians(30), 0.5));
        allPoints.add(new CurvePoint(0*COUNTS_PER_INCH , 0*COUNTS_PER_INCH,0.4,0.2 ,5*COUNTS_PER_INCH, Math.toRadians(30), 0.5));



        waitForStart();

        while (!isStopRequested()){
            u.globalCoordinatePositionUpdate();
            r.followCurve(allPoints, followAngles);

        }

    }

}
