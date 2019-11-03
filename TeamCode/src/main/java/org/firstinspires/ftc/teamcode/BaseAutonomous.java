package org.firstinspires.ftc.teamcode;


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

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


public abstract class BaseAutonomous extends LinearOpMode {

    HardwareSkyStone robot = null;

    private ElapsedTime runtime = new ElapsedTime();

    // define and initialize variables

    static final double     COUNTS_PER_MOTOR_REV    = 2240 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.5 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_CM       = 9.0 ;     // This measurement is more exact than inches
    static final double     COUNTS_PER_CM         = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * Math.PI));
    static final double     DRIVE_SPEED             = 0.9;
    static final double     TURBO_SPEED             = 0.7;
    static final double     TURN_SPEED              = 0.2;
    static final double     P_TURN_COEFF            = 0.03;
    static final double     HEADING_THRESHOLD       = 1 ;
    static final double     P_DRIVE_COEFF           = 0.03;

    private TFObjectDetector tfod;



    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
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

    boolean rackup = false;
    boolean dumbdriving = false;
    int remainingdistance = 10000;

    // initializes hardware; calibrates gyro
    public void inithardware(boolean test){

        robot = new HardwareSkyStone(test);
        robot.init(hardwareMap);
//        initializeObjectDetection();
        robot.setMode("encoders lmao");
        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData("Calibrating Gyro:", "Dont do anything");    //
        telemetry.update();
        if (!test) {
            // make sure the gyro is calibrated before continuing
            while (robot.realgyro.isCalibrating() /*|| robot.realgyro2.isCalibrating()*/) {
                sleep(300);
                idle();
            }
        }

        telemetry.addData(">", "haddi ready.");    //
        telemetry.update();


    }
    // NOT USED; gets average gyro value for more accurate angles
    public double getAverageGyro(){
        /*int sum = robot.realgyro.getIntegratedZValue() + robot.realgyro2.getIntegratedZValue();
        return sum/2;*/
        int sum = robot.realgyro.getIntegratedZValue();
        return sum;
    }

    // NOT USED (only for tank drive); uses gyro to move robot at a specific angle and distance (cm)
    public void gyroDrive ( double speed, double distance, double angle) {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  targetAvg;
        double  currentAvg;
        double  initialAvg;
        double  remainingCounts;
        double  countsTraveled;
        double  factor;
        double  deaccelerate;
        double  deaccelerate2;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            factor = 1;
            moveCounts = (int)(distance * COUNTS_PER_CM);
            newLeftTarget = robot.frontLeft.getCurrentPosition() + moveCounts;
            newRightTarget = robot.frontRight.getCurrentPosition() + moveCounts;
            targetAvg = (newLeftTarget + newRightTarget)/2;
            currentAvg = (robot.frontLeft.getCurrentPosition()+robot.frontRight.getCurrentPosition())/2 ;
            initialAvg = currentAvg;

            // Set Target and Turn On RUN_TO_POSITION
            robot.frontLeft.setTargetPosition(newLeftTarget);
            robot.frontRight.setTargetPosition(newRightTarget);

            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.frontLeft.setPower(speed);
            robot.frontRight.setPower(speed);


            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.frontLeft.isBusy() && robot.frontRight.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }
                currentAvg = (robot.frontLeft.getCurrentPosition()+robot.frontRight.getCurrentPosition())/2 ;
                remainingCounts = targetAvg - currentAvg;
                countsTraveled = currentAvg - initialAvg;

                if (countsTraveled < ACCELERATING_DISTANCE_COUNTS) {
                    factor = (countsTraveled * (1.0/ACCELERATING_DISTANCE_COUNTS)) + 0.1;
                    leftSpeed *= factor;
                    rightSpeed *= factor;
                    telemetry.addData("factor happened",leftSpeed);
                    telemetry.update();
                }

                if (remainingCounts <= BRAKING_DISTANCE_COUNTS) {
                    factor = remainingCounts * (1.0/BRAKING_DISTANCE_COUNTS);
                    leftSpeed *= factor;
                    rightSpeed *= factor;
                    telemetry.addData("factor happened",leftSpeed);
                    telemetry.update();
                }

                robot.frontLeft.setPower(leftSpeed);
                robot.frontRight.setPower(rightSpeed);

                // Display drive status for the driver.
//                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
//                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
//                telemetry.addData("Actual",  "%7d:%7d",      robot.frontLeft.getCurrentPosition(),
//                        robot.frontRight.getCurrentPosition());
//                telemetry.addData("Deaccelerate",  "%.2f"/*,      deaccelerate*/);
//                telemetry.addData("current_heading",getAverageGyro());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.addData("TargetAvg",targetAvg);
                telemetry.addData("CurrentAvg",currentAvg);
                telemetry.addData("factor",factor);



                telemetry.update();
            }

            // Stop all motion;
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);



            // Turn off RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    // Only for mecanum drive; uses gyro to move robot at a specific angle and distance (cm)
    public void gyroMecanumDrive ( double speed, double distance, double angle) {

        int     newFrontLeftTarget;
        int     newFrontRightTarget;
        int     newBackLeftTarget;
        int     newBackRightTarget;

        int     moveCounts;
        double  targetAvg;
        double  currentAvg;
        double  initialAvg;
        double  remainingCounts;
        double  countsTraveled;
        double  factor;

        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            factor = 1;
            moveCounts = (int)(distance * COUNTS_PER_CM);
            newFrontLeftTarget = robot.frontLeft.getCurrentPosition() + moveCounts;
            newFrontRightTarget = robot.frontRight.getCurrentPosition() + moveCounts;
            newBackLeftTarget = robot.backLeft.getCurrentPosition() + moveCounts;
            newBackRightTarget = robot.backRight.getCurrentPosition() + moveCounts;
            targetAvg = (newBackLeftTarget + newBackRightTarget + newFrontLeftTarget + newFrontRightTarget)/4;
            currentAvg = (robot.frontLeft.getCurrentPosition()+robot.frontRight.getCurrentPosition()+ robot.backLeft.getCurrentPosition()+robot.backRight.getCurrentPosition())/4 ;
            initialAvg = currentAvg;

            // Set Target and Turn On RUN_TO_POSITION
            robot.frontLeft.setTargetPosition(newFrontLeftTarget);
            robot.frontRight.setTargetPosition(newFrontRightTarget);
            robot.backRight.setTargetPosition(newBackRightTarget);
            robot.backLeft.setTargetPosition(newBackLeftTarget);

            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.frontLeft.setPower(speed);
            robot.frontRight.setPower(speed);
            robot.backRight.setPower(speed);
            robot.backLeft.setPower(speed);


            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() && (robot.frontLeft.isBusy() && robot.frontRight.isBusy() && robot.backLeft.isBusy() && robot.backRight.isBusy())  ) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }
                currentAvg = (robot.frontLeft.getCurrentPosition()+robot.frontRight.getCurrentPosition() + robot.backLeft.getCurrentPosition() + robot.backRight.getCurrentPosition())/4 ;
                remainingCounts = targetAvg - currentAvg;
                countsTraveled = currentAvg - initialAvg;


                if (countsTraveled < ACCELERATING_DISTANCE_COUNTS) {
                    factor = (countsTraveled * (1.0/ACCELERATING_DISTANCE_COUNTS)) + 0.1;
                    leftSpeed *= factor;
                    rightSpeed *= factor;
                    telemetry.addData("factor happened",leftSpeed);
                    telemetry.update();
                }

                if (remainingCounts <= BRAKING_DISTANCE_COUNTS) {
                    factor = remainingCounts * (1.0/BRAKING_DISTANCE_COUNTS);
                    leftSpeed *= factor;
                    rightSpeed *= factor;
                    telemetry.addData("factor happened",leftSpeed);
                    telemetry.update();
                }


                robot.frontLeft.setPower(leftSpeed);
                robot.frontRight.setPower(rightSpeed);


                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.addData("TargetAvg",targetAvg);
                telemetry.addData("CurrentAvg",currentAvg);
                telemetry.addData("factor",factor);



                telemetry.update();
            }

            // Stop all motion;
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);



            // Turn off RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
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

            telemetry.clear();
            telemetry.addData("move_x:",move_x);
            telemetry.addData("move_y:",move_y);
            telemetry.update();





            // reset the timeout time and start motion.
            runtime.reset();
            robot.frontLeft.setPower(Math.abs(wheels.getFrontLeftPower()*speed   ));
            robot.frontRight.setPower(Math.abs(wheels.getFrontRightPower()*speed ));
            robot.backRight.setPower(Math.abs(wheels.getRearRightPower()*speed    ));
            robot.backLeft.setPower(Math.abs(wheels.getRearLeftPower()*speed    ));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
                    (robot.frontLeft.isBusy() && robot.frontRight.isBusy() && robot.backLeft.isBusy() && robot.backRight.isBusy() )) {

                // Display it for the driver.
                //         telemetry.addData("Path1",  "Running to %7d :%7d", newFrontLeftTarget,  newFrontRightTarget);
                //       telemetry.addData("Path2",  "Running at %7d :%7d",
                //             robot.frontLeft.getCurrentPosition(),
                //           robot.frontRight.getCurrentPosition());

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

    // drive while pulling in hanger arm
    public void dumbencoderMecanumDrive(double speed, double distance , double timeoutS, double move_x, double move_y) {
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

            telemetry.clear();
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
            if (rackup){
                hangEncoder();
            }
            while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
                    (robot.frontLeft.isBusy() && robot.frontRight.isBusy() && robot.backLeft.isBusy() && robot.backRight.isBusy() )) {

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

    // get off hanger using encoder
    // direction should either be -1 or +1
    // +1 means you're dropping
    // -1 means you're hanging
    public void dropEncoder(int direction){
        int newTarget;

        newTarget = robot.hanger.getCurrentPosition() + (13000 * direction);
        robot.hanger.setTargetPosition(newTarget);
        robot.hanger.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.hanger.setPower(1);
        while (robot.hanger.isBusy()){
            telemetry.addData("haddi","buddi");
            telemetry.update();
        }
        robot.hanger.setPower(0);
        robot.hanger.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rackup = true;
    }

    // get on hanger using encoder
    public void hangEncoder(){
        int newTarget;

        newTarget = robot.hanger.getCurrentPosition() - remainingdistance;
        robot.hanger.setTargetPosition(newTarget);
        robot.hanger.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.hanger.setPower(1);
        while (robot.hanger.isBusy() && robot.frontLeft.isBusy() && robot.frontRight.isBusy() && robot.backLeft.isBusy() && robot.backRight.isBusy()){
            telemetry.addData("haddi","buddi");
            telemetry.update();
        }
        robot.hanger.setPower(0);
        robot.hanger.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (robot.hanger.getCurrentPosition() > newTarget){
            remainingdistance = robot.hanger.getCurrentPosition() - newTarget;
        }else{
            rackup = false;
            remainingdistance = 0;
        }
        telemetry.addData("remaining distance",remainingdistance);
        telemetry.update();
    }

    // NOT USED; uses encoders to drive - no gyro - tank drive
    public void encoderDrive(double speed, double leftCm, double rightCm, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.frontLeft.getCurrentPosition() + (int)(leftCm * COUNTS_PER_CM);
            newRightTarget = robot.frontRight.getCurrentPosition() + (int)(rightCm * COUNTS_PER_CM);
            robot.frontLeft.setTargetPosition(newLeftTarget);
            robot.frontRight.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.frontLeft.setPower(Math.abs(speed));
            robot.frontRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.frontLeft.isBusy() && robot.frontRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.frontLeft.getCurrentPosition(),
                        robot.frontRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    // turn to an angle using gyro
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.addData("current_heading", getAverageGyro());
            telemetry.update();
        }
    }

    // checks distance for gyroTurn/Drive - slows down when closer to end
    boolean onHeading(double speed, double angle, double PCoeff) {
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



    // makes sure lift power doesn't run on encoder
    public void setLiftPower(double power){
        robot.raiser.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.raiser.setPower(power);
    }

    // makes sure slide power doesn't run on encoder
    public void setSlidePower(double power){

        robot.slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.slider.setPower(power);

    }

    // NOT USED; stops all motors
    public void stopAllMotors() {
        robot.frontRight.setPower(0);
        robot.frontLeft.setPower(0);
        robot.raiser.setPower(0);

        robot.slider.setPower(0);
    }

    public void succ(long time) {
        robot.spinner.setPower(-1);
        robot.spinner2.setPower(-1);
        sleep(time);
        robot.spinner.setPower(0);
        robot.spinner2.setPower(0);

    }

    public void spit() {
        robot.spinner.setPower(1);
        robot.spinner2.setPower(1);
        sleep(1000);
        robot.spinner.setPower(0);
        robot.spinner2.setPower(0);

    }

    public void leftTurn(){
        robot.frontLeft.setPower(-1);
        robot.backLeft.setPower(-1);
        sleep(270);
        robot.frontLeft.setPower(0);
        robot.backLeft.setPower(0);
    }

    public void rightTurn(){
        robot.frontLeft.setPower(0.5);
        robot.backLeft.setPower(0.5);
        sleep(400);
        robot.frontLeft.setPower(0);
        robot.backLeft.setPower(0);
    }






    public void vuforiaJoint(){
        runtime.reset();
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

        // WARNING:
        // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
        // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
        // CONSEQUENTLY do not put any driving commands in this loop.
        // To restore the normal opmode structure, just un-comment the following line:

        // waitForStart();

        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.

        targetsSkyStone.activate();
        while (runtime.seconds()<5){
            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
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

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            }
            else {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();
        }

        targetsSkyStone.deactivate();
    }



}