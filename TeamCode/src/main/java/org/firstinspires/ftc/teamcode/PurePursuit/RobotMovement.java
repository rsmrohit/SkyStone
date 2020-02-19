package org.firstinspires.ftc.teamcode.PurePursuit;



import org.firstinspires.ftc.teamcode.HardwareSkyStone;
import org.firstinspires.ftc.teamcode.MecanumWheels;
import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;

import java.util.ArrayList;


public class RobotMovement {

    public int nextPointNum = 1;

    MecanumWheels w = new MecanumWheels();

    public OdometryGlobalCoordinatePosition globalPosition = null;
    HardwareSkyStone robot = null;


    public double pastX;
    public double pastY;

    public double xVelocity = 0;
    public double yVelocity = 0;

    public RobotMovement(OdometryGlobalCoordinatePosition g, HardwareSkyStone h){
        globalPosition = g;
        robot = h;
        pastX = g.getX();
        pastY = g.getY();
    }



    public void followCurve(ArrayList<CurvePoint> allPoints, double[] followAngles){

        updateVelocity();

        updateLocationAlongPath(allPoints, new Point(globalPosition.getX(), globalPosition.getY()));


        CurvePoint followMe = getFollowPointPath(allPoints, new Point(globalPosition.getX(), globalPosition.getY()), followAngles[nextPointNum]);


        goToPosition(followMe.x,followMe.y, followMe.moveSpeed, followAngles[nextPointNum], followMe.turnSpeed,followMe.slowDownTurnRadians,followMe.slowDownTurnAmount);
    }

    //Finds where the robot is along the path and updates nextPointNum to be the index of the next curve point
    public void updateLocationAlongPath(ArrayList<CurvePoint> allPoints, Point robotPos){
        double shortestDistanceToLine = 100000000;
        int smallest = 0;
        for (int i = 0; i < allPoints.size()-1;i++){
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

        if (smallest == nextPointNum){
            nextPointNum++;
        }

    }

    //Updates the x and y velocities of the robot and the past position
    public void updateVelocity(){
        xVelocity = globalPosition.getX() - pastX;
        yVelocity = globalPosition.getY() - pastY;
        pastX = globalPosition.getX();
        pastY = globalPosition.getY();
    }


    public CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, Point robotPos, double preferredAngle){

        CurvePoint followMe = new CurvePoint(pathPoints.get(nextPointNum));

        int topBound = nextPointNum+1;

        if (nextPointNum==2 && Math.hypot(pathPoints.get(2).x-robotPos.x,pathPoints.get(2).y-robotPos.y) >= followMe.followDistance){
            topBound = nextPointNum;
        }
        if (nextPointNum==8 && Math.hypot(pathPoints.get(8).x-robotPos.x,pathPoints.get(8).y-robotPos.y) >= followMe.followDistance){
            topBound = nextPointNum;
        }

        for (int i = nextPointNum-1; i < topBound && i <pathPoints.size()-1;i++){
            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(i+1);

            ArrayList<Point> intersections = MathFunctions.lineCircleIntersection(robotPos,followMe.followDistance, startLine.toPoint(),endLine.toPoint());


            double closestAngle = 100000000;
            for (Point thisIntersection: intersections){
                double angle = Math.atan2(thisIntersection.y-robotPos.y,thisIntersection.x-robotPos.x);
                double relativeAngle = MathFunctions.AngleWrap(angle - globalPosition.getOrientation() );
                double deltaAngle = Math.abs(MathFunctions.AngleWrap(relativeAngle - preferredAngle));

                if (deltaAngle < closestAngle){
                    closestAngle = deltaAngle;
                    followMe.setPoint(thisIntersection);
                    followMe.setCharacteristics(endLine);
                }
            }
        }
        if(nextPointNum==10 && Math.hypot(pathPoints.get(10).x-robotPos.x,pathPoints.get(10).y-robotPos.y) <= followMe.followDistance){
            followMe.turnSpeed = 0;
        }

        if (nextPointNum==11&& Math.hypot(pathPoints.get(11).x-robotPos.x,pathPoints.get(11).y-robotPos.y) <= followMe.followDistance){
            followMe.setPoint(new Point(robotPos.x,robotPos.y));
        }
        return followMe;
    }


    public void goToPosition(double x, double y, double movementSpeed, double preferedAngle, double turnSpeed, double slowDownTurnRadians, double slowDownAmount){

        double distanceToTarget = Math.hypot(x-globalPosition.getX(),y-globalPosition.getY());

        //Finds the angle to the target in the absolute xy coordinates of the map
        double absoluteAngleToTarget = Math.atan2(y-globalPosition.getY(),x-globalPosition.getX());

        //Finds the angle to the target in the xy coordinates of the robot
        double relativeAngle = MathFunctions.AngleWrap(absoluteAngleToTarget - globalPosition.getOrientation());


        double relativeXtoPoint = Math.cos(relativeAngle)*distanceToTarget;
        double relativeYtoPoint = Math.sin(relativeAngle)*distanceToTarget;

        double movementXPower = relativeXtoPoint/(Math.abs(relativeXtoPoint) + Math.abs(relativeYtoPoint));
        double movementYPower = relativeYtoPoint/(Math.abs(relativeXtoPoint) + Math.abs(relativeYtoPoint));

        double movement_x, movement_y, movement_turn;

        movement_x = movementXPower * movementSpeed;
        movement_y = movementYPower * movementSpeed;


        double relativeTurnAngle = relativeAngle - preferedAngle;

        if (relativeTurnAngle > slowDownTurnRadians){
            movement_y*=slowDownAmount;
            movement_x*=slowDownAmount;
        }

        if (distanceToTarget>=20){
            movement_turn = Range.clip(relativeTurnAngle/Math.toRadians(30),-1,1)* turnSpeed;
        }else{
            movement_turn = 0;
        }
        w.UpdateInput(movement_x,movement_y,movement_turn);
        robot.updateDriveTrainInputs(w);


    }

}

