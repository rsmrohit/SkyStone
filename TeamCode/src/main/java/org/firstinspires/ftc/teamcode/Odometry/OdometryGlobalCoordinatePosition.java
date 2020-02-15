package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/**
 * Created by Deez Nuts on 2/6/2020.
 */
public class OdometryGlobalCoordinatePosition implements Runnable{
    //Odometry wheels
    private DcMotor verticalEncoderLeft, verticalEncoderRight;
    private DcMotor horizontalEncoderLeft, horizontalEncoderRight;

    static final double offsetAngle = Math.PI/4;

    //Thead run condition
    private boolean isRunning = true;

    //Position variables used for storage and calculations
    double verticalRightEncoderWheelPosition = 0, verticalLeftEncoderWheelPosition = 0, changeInRobotOrientation = 0;
    double horizontalRightEncoderWheelPosition = 0, horizontalLeftEncoderWheelPosition = 0;
    private double robotGlobalXCoordinatePosition = 0, robotGlobalYCoordinatePosition = 0, robotOrientationRadians = 0;
    private double previousVerticalRightEncoderWheelPosition = 0, previousVerticalLeftEncoderWheelPosition = 0;
    private double previousHorizontalRightEncoderWheelPosition = 0, previousHorizontalLeftEncoderWheelPosition = 0;

    //Algorithm constants
    private double robotEncoderWheelDistance;


    //Sleep time interval (milliseconds) for the position update thread
    private int sleepTime;

    //Files to access the algorithm constants
    private File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");

    private int verticalLeftEncoderPositionMultiplier = 1;
    private int verticalRightEncoderPositionMultiplier = 1;
    private int normalEncoderPositionMultiplier = 1;

    /**
     * Constructor for GlobalCoordinatePosition Thread
     * @param verticalEncoderLeft left odometry encoder, facing the vertical direction
     * @param verticalEncoderRight right odometry encoder, facing the vertical direction
     * @param horizontalEncoderRight horizontal odometry encoder, perpendicular to the other two odometry encoder wheels
     * @param threadSleepDelay delay in milliseconds for the GlobalPositionUpdate thread (50-75 milliseconds is suggested)
     */
    public OdometryGlobalCoordinatePosition(DcMotor verticalEncoderLeft, DcMotor verticalEncoderRight, DcMotor horizontalEncoderLeft, DcMotor horizontalEncoderRight, double COUNTS_PER_INCH, int threadSleepDelay){
        this.verticalEncoderLeft = verticalEncoderLeft;
        this.verticalEncoderRight = verticalEncoderRight;
        this.horizontalEncoderLeft = horizontalEncoderLeft;
        this.horizontalEncoderRight = horizontalEncoderRight;
        sleepTime = threadSleepDelay;

        robotEncoderWheelDistance = Double.parseDouble(ReadWriteFile.readFile(wheelBaseSeparationFile).trim()) * COUNTS_PER_INCH;

    }

    /**
     * Updates the global (x, y, theta) coordinate position of the robot using the odometry encoders
     */
    private void globalCoordinatePositionUpdate(){
        //Get Current Positions
        verticalLeftEncoderWheelPosition = (verticalEncoderLeft.getCurrentPosition() * verticalLeftEncoderPositionMultiplier);
        verticalRightEncoderWheelPosition = (verticalEncoderRight.getCurrentPosition() * verticalRightEncoderPositionMultiplier);

        horizontalLeftEncoderWheelPosition = horizontalEncoderLeft.getCurrentPosition();
        horizontalRightEncoderWheelPosition = horizontalEncoderRight.getCurrentPosition();

        double leftChange = verticalLeftEncoderWheelPosition - previousVerticalLeftEncoderWheelPosition;
        double rightChange = verticalRightEncoderWheelPosition - previousVerticalRightEncoderWheelPosition;

        double leftChangeHorizontal = horizontalLeftEncoderWheelPosition - previousHorizontalLeftEncoderWheelPosition;
        double rightChangeHorizontal = horizontalRightEncoderWheelPosition - previousHorizontalRightEncoderWheelPosition;


        //Calculate Angle
        changeInRobotOrientation = (rightChange - leftChange) / (robotEncoderWheelDistance);
        robotOrientationRadians = ((robotOrientationRadians + changeInRobotOrientation));



        //Get the components of the motion


        double p = ((rightChange + leftChange) / 2);
        double n = ((leftChangeHorizontal+rightChangeHorizontal)/2);

        robotGlobalYCoordinatePosition += (p*Math.cos(offsetAngle + robotOrientationRadians) + n*Math.cos(offsetAngle-robotOrientationRadians));
        robotGlobalXCoordinatePosition += (-p*Math.sin(offsetAngle+robotOrientationRadians) + n*Math.sin(offsetAngle-robotOrientationRadians));

        //Code for odometers
        //robotGlobalXCoordinatePosition = robotGlobalXCoordinatePosition + (-p*Math.sin(robotOrientationRadians) + n*Math.cos(robotOrientationRadians));
        //robotGlobalYCoordinatePosition = robotGlobalYCoordinatePosition + (p*Math.cos(robotOrientationRadians) + n*Math.sin(robotOrientationRadians));

        previousVerticalLeftEncoderWheelPosition = verticalLeftEncoderWheelPosition;
        previousVerticalRightEncoderWheelPosition = verticalRightEncoderWheelPosition;
        previousHorizontalRightEncoderWheelPosition = horizontalRightEncoderWheelPosition;
        previousHorizontalLeftEncoderWheelPosition = horizontalLeftEncoderWheelPosition;
    }

    /**
     * Returns the robot's global x coordinate
     * @return global x coordinate
     */
    public double getX(){ return robotGlobalXCoordinatePosition; }

    /**
     * Returns the robot's global y coordinate
     * @return global y coordinate
     */
    public double getY(){ return robotGlobalYCoordinatePosition; }

    /**
     * Returns the robot's global orientation
     * @return global orientation, in degrees
     */
    public double getOrientation(){ return Math.toDegrees(robotOrientationRadians) % 360; }

    /**
     * Stops the position update thread
     */
    public void stop(){ isRunning = false; }

    public void reverseLeftEncoder(){
        if(verticalLeftEncoderPositionMultiplier == 1){
            verticalLeftEncoderPositionMultiplier = -1;
        }else{
            verticalLeftEncoderPositionMultiplier = 1;
        }
    }

    public void reverseRightEncoder(){
        if(verticalRightEncoderPositionMultiplier == 1){
            verticalRightEncoderPositionMultiplier = -1;
        }else{
            verticalRightEncoderPositionMultiplier = 1;
        }
    }

    public void reverseNormalEncoder(){
        if(normalEncoderPositionMultiplier == 1){
            normalEncoderPositionMultiplier = -1;
        }else{
            normalEncoderPositionMultiplier = 1;
        }
    }

    /**
     * Runs the thread
     */
    @Override
    public void run() {
        while(isRunning) {
            globalCoordinatePositionUpdate();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
