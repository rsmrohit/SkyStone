package org.firstinspires.ftc.teamcode.PurePursuit;


import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareSkyStone;
import org.firstinspires.ftc.teamcode.MecanumWheels;
import org.firstinspires.ftc.teamcode.Odometry.UpdateBoi;

import java.util.ArrayList;


public class RobotMovement {

    public int nextPointNum = 1;

    MecanumWheels w = new MecanumWheels();

    public UpdateBoi updateBoi = null;
    HardwareSkyStone robot = null;

    boolean f = false;
    ElapsedTime elapsedTime = new ElapsedTime();
    double pastTime = elapsedTime.seconds();


    public double pastX = 0;
    public double pastY = 0;

    public double xVelocity = 0;
    public double yVelocity = 0;

    public Telemetry telemetry;

    public RobotMovement(UpdateBoi u, HardwareSkyStone h, Telemetry t){
        updateBoi = u;
        robot = h;
        pastX = u.getX();
        pastY = u.getY();
        telemetry = t;
    }



    public void followCurve(ArrayList<CurvePoint> allPoints, double[] followAngles){

        updateVelocity();

        updateLocationAlongPath(allPoints, new Point(updateBoi.getX(), updateBoi.getY()));


        CurvePoint followMe = getFollowPointPath(allPoints, new Point(updateBoi.getX(), updateBoi.getY()), followAngles[nextPointNum]);


        goToPosition(followMe.x,followMe.y, followMe.moveSpeed, followAngles[nextPointNum], followMe.turnSpeed,followMe.slowDownTurnRadians,followMe.slowDownTurnAmount);
    }

    //Finds where the robot is along the path and updates nextPointNum to be the index of the next curve point
    public void updateLocationAlongPath(ArrayList<CurvePoint> allPoints, Point robotPos){
        double shortestDistanceToLine = 100000000;
        int smallest = 0;
        for (int i = (nextPointNum-1); i < allPoints.size()-1 && i < nextPointNum+5;i++){
            double[] dArray = MathFunctions.pointLineIntersection(robotPos,allPoints.get(i),allPoints.get(i+1));

            double dist = dArray[0];

            double lineAngle = Math.atan2(allPoints.get(i+1).y-allPoints.get(i).y,allPoints.get(i+1).x-allPoints.get(i).x);

            double robotAngle = Math.atan2(yVelocity,xVelocity);

            double directionDiff = Math.abs(MathFunctions.AngleWrap(robotAngle-lineAngle));



            if (dist <= shortestDistanceToLine && directionDiff < Math.toRadians(130)){
                shortestDistanceToLine = dist;
                smallest = i;

            }

        }

        if (smallest ==3 && !f){

            f = true;
        }

        if (smallest == nextPointNum){
            nextPointNum++;
            telemetry.addData("dist past","%.3f",MathFunctions.pointLineIntersection(robotPos,allPoints.get(smallest-1),allPoints.get(smallest))[0]);
            telemetry.addData("dist 4","%.3f",MathFunctions.pointLineIntersection(robotPos,allPoints.get(smallest),allPoints.get(nextPointNum))[0]);
            double line = Math.atan2(allPoints.get(smallest).y-allPoints.get(smallest-1).y,allPoints.get(smallest).x-allPoints.get(smallest-1).x);

            telemetry.addData("line angle", line);
            telemetry.addData("robot angle",Math.atan2(yVelocity,xVelocity));

            telemetry.update();
        }



    }

    //Updates the x and y velocities of the robot and the past position
    public void updateVelocity(){
        double delta = elapsedTime.seconds()-pastTime;

        xVelocity = (updateBoi.getX() - pastX)/delta;
        yVelocity = (updateBoi.getY() - pastY)/delta;
        if (Math.abs(xVelocity)<10){
            xVelocity=0;
        }
        if (Math.abs(yVelocity)<10){
            yVelocity=0;
        }

        pastX = updateBoi.getX();
        pastY = updateBoi.getY();
        pastTime = elapsedTime.seconds();

    }


    public CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, Point robotPos, double preferredAngle){

        CurvePoint followMe = new CurvePoint(pathPoints.get(nextPointNum));

        int topBound = nextPointNum+2;



        for (int i = nextPointNum-1; i < topBound && i <pathPoints.size()-1;i++){
            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(i+1);

            ArrayList<Point> intersections = MathFunctions.lineCircleIntersection(robotPos,followMe.followDistance, startLine.toPoint(),endLine.toPoint());


            double closestAngle = 100000000;
            for (Point thisIntersection: intersections){
                double angle = Math.atan2(thisIntersection.y-robotPos.y,thisIntersection.x-robotPos.x);

                double relativeAngle = MathFunctions.AngleWrap(angle - (updateBoi.getOrientation() + Math.toRadians(90) ) );
                double deltaAngle = Math.abs(MathFunctions.AngleWrap(relativeAngle - preferredAngle));


                if (deltaAngle < closestAngle){
                    closestAngle = deltaAngle;
                    followMe.setPoint(thisIntersection);
                    followMe.setCharacteristics(endLine);
                }
            }
        }


        return followMe;

    }


    public void goToPosition(double x, double y, double movementSpeed, double preferedAngle, double turnSpeed, double slowDownTurnRadians, double slowDownAmount){

        double distanceToTarget = Math.hypot(x-updateBoi.getX(),y-updateBoi.getY());

        //Finds the angle to the target in the absolute xy coordinates of the map
        double absoluteAngleToTarget = Math.atan2(y-updateBoi.getY(),x-updateBoi.getX());

        //Finds the angle to the target in the xy coordinates of the robot
        double relativeAngle = MathFunctions.AngleWrap(absoluteAngleToTarget - updateBoi.getOrientation() );

        double relativeXtoPoint = Math.cos(relativeAngle)*distanceToTarget;
        double relativeYtoPoint = Math.sin(relativeAngle)*distanceToTarget;


        double magnitude = Math.hypot(relativeXtoPoint,relativeYtoPoint);
        double movementXPower = relativeXtoPoint/magnitude;
        double movementYPower = relativeYtoPoint/magnitude;

        double movement_x, movement_y, movement_turn;

        movement_x = movementXPower * movementSpeed;
        movement_y = movementYPower * movementSpeed;


        double relativeTurnAngle = relativeAngle - preferedAngle;

        if (Math.abs(relativeTurnAngle) > slowDownTurnRadians){
            movement_y*=slowDownAmount;
            movement_x*=slowDownAmount;
        }

        if (distanceToTarget>=10){
            movement_turn = Range.clip(relativeTurnAngle/slowDownTurnRadians,-1,1)* turnSpeed;
        }else{
            movement_turn = 0;
        }


        w.UpdateInput(movement_x,movement_y,movement_turn);

        robot.updateDriveTrainInputs(w);


    }

}

