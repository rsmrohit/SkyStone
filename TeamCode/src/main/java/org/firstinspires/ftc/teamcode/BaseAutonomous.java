package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Odometry.UpdateBoi;
import org.firstinspires.ftc.teamcode.PurePursuit.MathFunctions;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


public abstract class BaseAutonomous extends LinearOpMode {

    HardwareSkyStone robot = null;

    ElapsedTime runtime = new ElapsedTime();

    // define and initialize variables

    static final double     COUNTS_PER_MOTOR_REV    = 767.2 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_CM       = 10.0 ;     // This measurement is more exact than inches
    static final double     COUNTS_PER_CM         = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * Math.PI));
    static final double     DRIVE_SPEED             = 0.9;
    static final double     TURBO_SPEED             = 0.7;
    static final double     TURN_SPEED              = 0.3;
    static final double     P_TURN_COEFF            = 0.03;
    static final double     HEADING_THRESHOLD       = 1 ;
    static final double     P_DRIVE_COEFF           = 0.03;
    static final double     CORRECTION              = Math.sqrt(2);

    final double COUNTS_PER_INCH = 307.699557;

    private TFObjectDetector tfod;
    boolean dumbdriving;

    public VuforiaTrackables haddi;
    public List<VuforiaTrackable> buddi;

    public UpdateBoi updateBoi;

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;


    private static final String VUFORIA_KEY =
            "AW96LTb/////AAABmaivtHzrd0gju5XtetpwppuGSyfDkXdWv7vyrqddGgyRP4m7UbrjLIwGi6O3SJkxFrMRLkdY527rsPR9bL89cstIHSsGMBN04yqphi/q9ce+NEG/qgv3P6e4MNoT3HzlMPUvQZjs4QPsRENnKZqHcru2L//SMz7PiX0juTXm695WBa5j2W3neYQ15sdtx1ZH58q7q5vdFsZGP7+D1PD5IUBOLn8noSkZF5gaGqbmJ3YxcYIYHHl6GsWZ0ff8X/VtGgh+pWkxeZlsyPhXzRqqTC1/NyYgm+umEI0gEAjwL5Mqi7bdDMrXIADKw1rSJNm+ivmrNtceobxkjTovWhqdoLQolQoAuTszTQUuorbzurqI";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;


    static final double  BRAKING_DISTANCE_COUNTS  = (40 * COUNTS_PER_CM);
    static final double  ACCELERATING_DISTANCE_COUNTS  = (20 * COUNTS_PER_CM);



    // initializes hardware  and vuforia; calibrates gyro
    public void inithardware(boolean test){

        robot = new HardwareSkyStone(test);

        robot.init(hardwareMap);

        initVuforia();
        robot.setMode("encoders lmao");
        //UpdateBoi u = new UpdateBoi(robot.verticalLeft, robot.verticalRight, robot.horizontal, COUNTS_PER_INCH, robot.imu);
        telemetry.addData("robot", "initialized.");    //
        telemetry.update();


    }
    // NOT USED; gets average gyro value for more accurate angles
    public double getAverageGyro(){
        return robot.imu.getAngularOrientation().firstAngle;

    }


    public void goToPosition(double x, double y, double movementSpeed, double accelerateDistance, double preferedAngle, double turnSpeed, double slowDownTurnRadians, double slowDownAmount){

        double distanceToTarget = Math.hypot(x-updateBoi.getX(),y-updateBoi.getY());
        double initialD = distanceToTarget;
        MecanumWheels w = new MecanumWheels();
        double acceleration = (movementSpeed*movementSpeed)/(2*accelerateDistance);
        double multiplier = 0;
        double pastTime = runtime.seconds();
        while (distanceToTarget > 4*COUNTS_PER_INCH){
            updateBoi.globalCoordinatePositionUpdate();
            distanceToTarget =  Math.hypot(x-updateBoi.getX(),y-updateBoi.getY());
            //Finds the angle to the target in the absolute xy coordinates of the map
            double absoluteAngleToTarget = Math.atan2(y-updateBoi.getY(),x-updateBoi.getX());

            //Finds the angle to the target in the xy coordinates of the robot
            double relativeAngle = MathFunctions.AngleWrap(absoluteAngleToTarget - updateBoi.getOrientation() );

            double relativeXtoPoint = Math.cos(relativeAngle)*distanceToTarget;
            double relativeYtoPoint = Math.sin(relativeAngle)*distanceToTarget;


            //Comment out these lines are replace the multiplier down below with movement speeed if you don't want acceleration/deceleration
            if (initialD-distanceToTarget < accelerateDistance && multiplier < movementSpeed){
                multiplier+= acceleration*(runtime.seconds()-pastTime);
            }else if (distanceToTarget < (accelerateDistance + 4*COUNTS_PER_INCH) && multiplier > 0){
                multiplier-= acceleration*(runtime.seconds()-pastTime);
            } else {
                telemetry.addData("top speed", multiplier);
                telemetry.update();
            }
            pastTime = runtime.seconds();




            double magnitude = Math.hypot(relativeXtoPoint,relativeYtoPoint);
            double movementXPower = relativeXtoPoint/magnitude;
            double movementYPower = relativeYtoPoint/magnitude;

            double movement_x, movement_y, movement_turn;

            movement_x = movementXPower * multiplier;
            movement_y = movementYPower * multiplier;


            double relativeTurnAngle = relativeAngle - preferedAngle;

            if (Math.abs(relativeTurnAngle) > slowDownTurnRadians){
                movement_y*=slowDownAmount;
                movement_x*=slowDownAmount;
            }

            if (distanceToTarget>=10){
                movement_turn = org.firstinspires.ftc.teamcode.PurePursuit.Range.clip(relativeTurnAngle/slowDownTurnRadians,-1,1)* turnSpeed;
            }else{
                movement_turn = 0;
            }


            w.UpdateInput(movement_x,movement_y,movement_turn);

            robot.updateDriveTrainInputs(w);

        }
        robot.updateDriveTrainInputs(0,0,0,0);

    }


    // checks to make sure that all motors which should be running are running
    public boolean areMotorsRunning(MecanumWheels wheels){
        for (int i = 0; i < wheels.wheelPowers.length;i++){
            if (wheels.wheelPowers[i]!=0){
                switch (i){
                    case 0:
                        if (!robot.frontLeft.isBusy()) {
                            telemetry.addData("front left motor", "done");
                            telemetry.update();

                            return false;
                        }
                        break;
                    case 1:
                        if (!robot.frontRight.isBusy()) {
                            telemetry.addData("front right power",  wheels.wheelPowers[i]);
                            telemetry.addData("front right motor", "done");
                            telemetry.update();

                            return false;
                        }
                        break;
                    case 2:
                        if (!robot.backLeft.isBusy()){
                            telemetry.addData("back left motor", "done");
                            telemetry.update();

                            return false;
                        }
                        break;
                    case 3:
                        if (!robot.backRight.isBusy()) {
                            telemetry.addData("back right motor", "done");
                            telemetry.update();

                            return false;
                        }
                        break;
                }
            }
        }
        return true;
    }


    // simple directional drive function for a mecanum drive train
    public void encoderMecanumDrive(double speed, double distance , double timeoutS, double move_x, double move_y) {
        int     newFrontLeftTarget;
        int     newFrontRightTarget;
        int     newBackLeftTarget;
        int     newBackRightTarget;
        int     frontLeftSign;
        int     frontRightSign;
        int     backLeftSign;
        int     backRightSign;
        MecanumWheels wheels = new MecanumWheels();

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            wheels.UpdateInput(move_x, move_y, 0);


            frontLeftSign = (int) ( Math.abs(wheels.getFrontLeftPower()) /wheels.getFrontLeftPower());
            frontRightSign = (int) ( Math.abs(wheels.getFrontRightPower()) /wheels.getFrontRightPower());
            backLeftSign = (int) ( Math.abs(wheels.getRearLeftPower() ) /wheels.getRearLeftPower());
            backRightSign = (int) ( Math.abs(wheels.getRearRightPower()) /wheels.getRearRightPower());



            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = robot.frontLeft.getCurrentPosition() + (int)(distance * COUNTS_PER_CM*frontLeftSign);
            newBackLeftTarget = robot.backLeft.getCurrentPosition() + (int)(distance * COUNTS_PER_CM*backLeftSign);
            newFrontRightTarget = robot.frontRight.getCurrentPosition() + (int)(distance * COUNTS_PER_CM*frontRightSign);
            newBackRightTarget = robot.backRight.getCurrentPosition() + (int)(distance * COUNTS_PER_CM*backRightSign);


            //Set target position
            robot.frontLeft.setTargetPosition(newFrontLeftTarget);
            robot.frontRight.setTargetPosition(newFrontRightTarget);
            robot.backLeft.setTargetPosition(newBackLeftTarget);
            robot.backRight.setTargetPosition(newBackRightTarget);



            // Turn On RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.frontLeft.setPower(Math.abs(wheels.getFrontLeftPower()*speed));
            robot.frontRight.setPower(Math.abs(wheels.getFrontRightPower()*speed));
            robot.backRight.setPower(Math.abs(wheels.getRearRightPower()*speed));
            robot.backLeft.setPower(Math.abs(wheels.getRearLeftPower()*speed));



            while (opModeIsActive() && (runtime.seconds() < timeoutS) && areMotorsRunning(wheels) && !isStopRequested()) {

                telemetry.addData("FrontLeftPower",robot.frontLeft.getPower());
                telemetry.addData("FrontRightPower",robot.frontRight.getPower());
                telemetry.addData("BackRightPower",robot.backRight.getPower());
                telemetry.addData("BackLeftPower",robot.backLeft.getPower());

                telemetry.update();
            }

            // Stop all motion;
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backRight.setPower(0);
            robot.backLeft.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    public void bencoderMecanumDrive(double speed, double distance , double timeoutS, double move_x, double move_y) {
        int     newFrontLeftTarget;
        int     newFrontRightTarget;
        int     newBackLeftTarget;
        int     newBackRightTarget;
        int     frontLeftSign;
        int     frontRightSign;
        int     backLeftSign;
        int     backRightSign;
        boolean once = false;
        MecanumWheels wheels = new MecanumWheels();

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            wheels.UpdateInput(move_x, move_y, 0);


            frontLeftSign = (int) ( Math.abs(wheels.getFrontLeftPower()) /wheels.getFrontLeftPower());
            frontRightSign = (int) ( Math.abs(wheels.getFrontRightPower()) /wheels.getFrontRightPower());
            backLeftSign = (int) ( Math.abs(wheels.getRearLeftPower() ) /wheels.getRearLeftPower());
            backRightSign = (int) ( Math.abs(wheels.getRearRightPower()) /wheels.getRearRightPower());



            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = robot.frontLeft.getCurrentPosition() + (int)(distance * COUNTS_PER_CM*frontLeftSign);
            newBackLeftTarget = robot.backLeft.getCurrentPosition() + (int)(distance * COUNTS_PER_CM*backLeftSign);
            newFrontRightTarget = robot.frontRight.getCurrentPosition() + (int)(distance * COUNTS_PER_CM*frontRightSign);
            newBackRightTarget = robot.backRight.getCurrentPosition() + (int)(distance * COUNTS_PER_CM*backRightSign);


            //Set target position
            robot.frontLeft.setTargetPosition(newFrontLeftTarget);
            robot.frontRight.setTargetPosition(newFrontRightTarget);
            robot.backLeft.setTargetPosition(newBackLeftTarget);
            robot.backRight.setTargetPosition(newBackRightTarget);



            // Turn On RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.frontLeft.setPower(Math.abs(wheels.getFrontLeftPower()*speed));
            robot.frontRight.setPower(Math.abs(wheels.getFrontRightPower()*speed));
            robot.backRight.setPower(Math.abs(wheels.getRearRightPower()*speed));
            robot.backLeft.setPower(Math.abs(wheels.getRearLeftPower()*speed));



            while (opModeIsActive() && (runtime.seconds() < timeoutS) && areMotorsRunning(wheels) && !isStopRequested()) {
                if (Math.abs(robot.frontLeft.getCurrentPosition()-robot.frontLeft.getTargetPosition()) < 100 && !once){
                    once = true;
                    robot.turnoright.setPosition(0);
                }
            }

            // Stop all motion;
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backRight.setPower(0);
            robot.backLeft.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    // function to test out specified motor
    public void testdrive(double speed, double distance, DcMotor input ) {

        int newTarget;

        MecanumWheels wheels = new MecanumWheels();

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            wheels.UpdateInput(0, 1, 0);

            // Determine new target position, and pass to motor controller

            newTarget = input.getCurrentPosition() + (int)(distance * COUNTS_PER_CM);

            input.setTargetPosition(newTarget);

            // reset the timeout time and start motion.
            runtime.reset();

            input.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            input.setPower(wheels.getRearRightPower()*speed);



            while (opModeIsActive()  && (input.isBusy())) {


                telemetry.addData("motor power",input.getPower());
                telemetry.addData("current position",input.getCurrentPosition());

                telemetry.update();
            }

            // Stop all motion;
            input.setPower(0);



            // Turn off RUN_TO_POSITION
            input.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }


    // turn to an angle using gyro
    public void gyroTurn (  double speed, double angle, double threshold) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF, threshold)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.addData("current_heading", getAverageGyro());
            telemetry.update();
        }
    }


    public void gyroCurve( double speed, double angle, double x, double y){
        while (opModeIsActive() && !onHeadingish(speed, angle, P_TURN_COEFF,x,y)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.addData("current_heading", getAverageGyro());
            telemetry.update();
        }
    }

    public void gyroTurnAndMove( double speed, double angle, double ratio, double intendedDirection){

        while (opModeIsActive() && !onHeadingbruh(speed, angle, P_TURN_COEFF,ratio,intendedDirection)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.addData("current_heading", getAverageGyro());
            telemetry.update();
        }
    }

    boolean onHeadingbruh(double speed, double angle, double PCoeff, double ratio, double intendedDirection){
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;


        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        double delta = robot.imu.getAngularOrientation().firstAngle-intendedDirection;
        double deltaRad = delta*Math.PI/180;
        double x = ratio*Math.sin(deltaRad);
        double y = ratio*Math.cos(deltaRad);

        MecanumWheels wheels = new MecanumWheels(x,y,rightSpeed);

        // Send desired speeds to motors.
        robot.frontLeft.setPower(wheels.getFrontLeftPower());
        robot.frontRight.setPower(wheels.getFrontRightPower());
        robot.backLeft.setPower(wheels.getRearLeftPower());
        robot.backRight.setPower(wheels.getRearRightPower());


        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    boolean onHeadingish(double speed, double angle, double PCoeff, double x, double y){
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;


        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= 20) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        MecanumWheels wheels = new MecanumWheels(x,y,rightSpeed);

        // Send desired speeds to motors.
        robot.frontLeft.setPower(wheels.getFrontLeftPower());
        robot.frontRight.setPower(wheels.getFrontRightPower());
        robot.backLeft.setPower(wheels.getRearLeftPower());
        robot.backRight.setPower(wheels.getRearRightPower());


        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }



    // checks distance for gyroTurn/Drive - slows down when closer to end
    boolean onHeading(double speed, double angle, double PCoeff, double threshold) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= threshold) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.frontLeft.setPower(leftSpeed);
        robot.frontRight.setPower(rightSpeed);
        robot.backLeft.setPower(leftSpeed);
        robot.backRight.setPower(rightSpeed);


        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    //show the error in gyro angle
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - getAverageGyro();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }


    public void succ() {
        robot.spinner.setPower(1);
        robot.spinner2.setPower(1);

    }
    public void succstop(){
        robot.spinner.setPower(0);
        robot.spinner2.setPower(0);
    }

    public void spit() {
        robot.spinner.setPower(-1);
        robot.spinner2.setPower(-1);


    }

    public void grab(){
        robot.rightclaw.setPower(1);
        robot.leftclaw.setPower(1);
        sleep(800);
    }


    public void release(){
        robot.rightclaw.setPower(-1);
        robot.leftclaw.setPower(-1);
        sleep(550);
        robot.rightclaw.setPower(0);
        robot.leftclaw.setPower(0);

    }

    public void rightClamp(){
        robot.extendoright.setPosition(0.45);
    }

    public void bruhhh(){
        robot.extendoright.setPosition(0.5);
        sleep(100);
        robot.turnoright.setPosition(0.48);
    }
    public void liftClamp(){
        robot.extendoright.setPosition(0.1);
    }

    public void dropThaBlock(){
        robot.extendoright.setPosition(0.25);
        robot.turnoright.setPosition(0);
        sleep(300);
    }

    public void dumbencoderMecanumDrive(double speed, double distance , double timeoutS, double move_x, double move_y, boolean out) {
        int     newFrontLeftTarget;
        int     newFrontRightTarget;
        int     newBackLeftTarget;
        int     newBackRightTarget;
        int     frontLeftSign;
        int     frontRightSign;
        int     backLeftSign;
        int     backRightSign;
        MecanumWheels wheels = new MecanumWheels();

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            wheels.UpdateInput(move_x, move_y, 0);

            frontLeftSign = (int) (Math.abs(wheels.getFrontLeftPower())/wheels.getFrontLeftPower());
            frontRightSign = (int) (Math.abs(wheels.getFrontRightPower())/wheels.getFrontRightPower());
            backLeftSign = (int) (Math.abs(wheels.getRearLeftPower()) /wheels.getRearLeftPower());
            backRightSign = (int) (Math.abs(wheels.getRearRightPower())/wheels.getRearRightPower());

            telemetry.addData("fl",frontLeftSign);
            telemetry.addData("fr",frontRightSign);
            telemetry.addData("bl",backLeftSign);
            telemetry.addData("br",backRightSign);
            telemetry.update();


            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = robot.frontLeft.getCurrentPosition() + (int)(distance * COUNTS_PER_CM*frontLeftSign);
            newBackLeftTarget = robot.backLeft.getCurrentPosition() + (int)(distance * COUNTS_PER_CM*backLeftSign);

            newFrontRightTarget = robot.frontRight.getCurrentPosition() + (int)(distance * COUNTS_PER_CM*frontRightSign);
            newBackRightTarget = robot.backRight.getCurrentPosition() + (int)(distance * COUNTS_PER_CM*backRightSign);


            robot.frontLeft.setTargetPosition(newFrontLeftTarget);
            robot.frontRight.setTargetPosition(newFrontRightTarget);
            robot.backLeft.setTargetPosition(newBackLeftTarget);
            robot.backRight.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            telemetry.addData("move_x:",move_x);
            telemetry.addData("move_y:",move_y);
            telemetry.update();


            // reset the timeout time and start motion.
            runtime.reset();
            robot.frontLeft.setPower(Math.abs(wheels.getFrontLeftPower()*speed   ));
            robot.frontRight.setPower(Math.abs(wheels.getFrontRightPower()*speed ));
            robot.backRight.setPower(Math.abs(wheels.getRearRightPower()*speed    ));
            robot.backLeft.setPower(Math.abs(wheels.getRearLeftPower()*speed    ));

            dumbdriving = true;

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.

            horizontalEncoder(out,wheels);

            while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
                    (areMotorsRunning(wheels) )) {

                telemetry.addData("running","yes");
                telemetry.update();

            }

            dumbdriving = false;
            // Stop all motion;
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backRight.setPower(0);
            robot.backLeft.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        }
    }

    public void verticalEncoder(int target){
        robot.verticalSlider.setTargetPosition(target);
        robot.verticalSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.verticalSlider.setPower(1);
        while (robot.verticalSlider.isBusy()){

        }
        robot.verticalSlider.setPower(0);
        robot.verticalSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void bruh(){
        ElapsedTime t = new ElapsedTime();
        t.reset();
        robot.verticalSlider.setPower(1);

        while (t.seconds()< 1.0 && robot.verticalSlider.getCurrentPosition()<400){
            telemetry.addData("vertical position",robot.verticalSlider.getCurrentPosition());
            telemetry.update();
        }
        robot.verticalSlider.setPower(0);


        robot.horizontalSlider.setPower(1);
        t.reset();
        while (t.seconds()<1.5 && robot.horizontalSlider.getCurrentPosition()<500){

        }

        robot.horizontalSlider.setPower(0);

        robot.verticalSlider.setPower(-1);
        t.reset();
        while (t.seconds()<1.0 && robot.verticalSlider.getCurrentPosition() > 150){

        }
        robot.verticalSlider.setPower(0);

    }

    public void bruhbuddi(){
        ElapsedTime t = new ElapsedTime();
        t.reset();
        robot.verticalSlider.setPower(1);

        while (t.seconds()<2.0 && robot.verticalSlider.getCurrentPosition()<800){
            telemetry.addData("vertical position",robot.verticalSlider.getCurrentPosition());
            telemetry.update();
        }

        robot.verticalSlider.setPower(0);



        robot.horizontalSlider.setPower(1);
        t.reset();
        while (t.seconds()<1.5 && robot.horizontalSlider.getCurrentPosition()<500){

        }

        robot.horizontalSlider.setPower(0);
        robot.clamper.setPosition(0.03);

        robot.horizontalSlider.setPower(-1);
        t.reset();
        while (t.seconds()<1.5 && robot.horizontalSlider.getCurrentPosition()>100){

        }

        robot.horizontalSlider.setPower(0);

        robot.verticalSlider.setPower(-1);
        t.reset();
        while (t.seconds()<2 && robot.verticalSlider.getCurrentPosition() > 150){

        }
        robot.verticalSlider.setPower(0);

    }

    public void upandOut(){

        ElapsedTime t = new ElapsedTime();
        t.reset();


        robot.verticalSlider.setTargetPosition(350);
        robot.verticalSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.verticalSlider.setPower(-1);


        robot.horizontalSlider.setTargetPosition(320);
        robot.horizontalSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.horizontalSlider.setPower(1);
        while(robot.horizontalSlider.isBusy() && t.seconds()<1.5){
            telemetry.addData("horizontal position",robot.horizontalSlider.getCurrentPosition());
            telemetry.update();

        }



        robot.horizontalSlider.setPower(0);
        robot.horizontalSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        robot.verticalSlider.setTargetPosition(0);
        robot.verticalSlider.setPower(1);

        while (robot.verticalSlider.isBusy()){

        }
        robot.verticalSlider.setPower(0);
        robot.verticalSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



    }

    public void horizontalEncoder(boolean out, MecanumWheels wheels) {

        if (out) {
//            robot.clamper.setPosition(0.2);
            robot.horizontalSlider.setTargetPosition(300);
        } else {
            robot.horizontalSlider.setTargetPosition(0);
        }

        robot.horizontalSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.horizontalSlider.setPower(1);
        while(robot.horizontalSlider.isBusy() && areMotorsRunning(wheels)){
            telemetry.addData("dumb drive","succing");
            telemetry.addData("current position",robot.horizontalSlider.getCurrentPosition());
            telemetry.update();

        }
        robot.horizontalSlider.setPower(0);
        robot.horizontalSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void bruhturn(int direction){
        robot.updateDriveTrainInputs(direction,-direction,direction,-direction);
        sleep(100);
        robot.updateDriveTrainInputs(0,0,0,0);
    }



    public void initVuforia(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }
        haddi = targetsSkyStone;
        buddi = allTrackables;
        targetsSkyStone.activate();
    }


    public String vuforiaJoint(VuforiaTrackables targetsSkyStone, List<VuforiaTrackable> allTrackables){
        runtime.reset();
        com.vuforia.CameraDevice.getInstance().setFlashTorchMode(true);

        com.vuforia.CameraDevice.getInstance().setField("opti-zoom","opti-zoom-on");
        com.vuforia.CameraDevice.getInstance().setField("zoom","25");

        while (runtime.seconds()<3 && !isStopRequested()){
            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {

                    if (trackable.getName().equals("Stone Target")){
                        targetVisible = true;

                        // getUpdatedRobotLocation() will return null if no new information is available since
                        // the last time that call was made, or if the trackable is not currently visible.
                        OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                        if (robotLocationTransform != null) {
                            lastLocation = robotLocationTransform;
                        }
                        break;
                    }
                }

            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
                if (translation.get(1)/mmPerInch < -3.75){
                    return "Left";
                } else if (translation.get(1)/mmPerInch < 2.8){
                    return "Center";
                } else {
                    return "Right";
                }

            }
            else {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();
        }

        targetsSkyStone.deactivate();
        return "Left";
    }

    public String vuforiaJointo(VuforiaTrackables targetsSkyStone, List<VuforiaTrackable> allTrackables){
        runtime.reset();

        com.vuforia.CameraDevice.getInstance().setFlashTorchMode(true);

        com.vuforia.CameraDevice.getInstance().setField("opti-zoom","opti-zoom-on");
        com.vuforia.CameraDevice.getInstance().setField("zoom","25");


        while (runtime.seconds()<1.5 && !isStopRequested()){
            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {

                    if (trackable.getName().equals("Stone Target")){
                        targetVisible = true;

                        // getUpdatedRobotLocation() will return null if no new information is available since
                        // the last time that call was made, or if the trackable is not currently visible.
                        OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                        if (robotLocationTransform != null) {
                            lastLocation = robotLocationTransform;
                        }
                        break;
                    }
                }

            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
                if (translation.get(1)/mmPerInch < -4.45){
                    return "Left";
                } else if (translation.get(1)/mmPerInch < 2.9){
                    return "Center" ;
                } else {
                    return "Right";
                }

            }
            else {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();
        }

        targetsSkyStone.deactivate();
        return "Right";
    }


}