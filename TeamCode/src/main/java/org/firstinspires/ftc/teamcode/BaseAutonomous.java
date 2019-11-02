package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import java.util.List;


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
    private VuforiaLocalizer vuforia;
    private static final String VUFORIA_KEY =
            "AW96LTb/////AAABmaivtHzrd0gju5XtetpwppuGSyfDkXdWv7vyrqddGgyRP4m7UbrjLIwGi6O3SJkxFrMRLkdY527rsPR9bL89cstIHSsGMBN04yqphi/q9ce+NEG/qgv3P6e4MNoT3HzlMPUvQZjs4QPsRENnKZqHcru2L//SMz7PiX0juTXm695WBa5j2W3neYQ15sdtx1ZH58q7q5vdFsZGP7+D1PD5IUBOLn8noSkZF5gaGqbmJ3YxcYIYHHl6GsWZ0ff8X/VtGgh+pWkxeZlsyPhXzRqqTC1/NyYgm+umEI0gEAjwL5Mqi7bdDMrXIADKw1rSJNm+ivmrNtceobxkjTovWhqdoLQolQoAuTszTQUuorbzurqI";
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";


    static final double  BRAKING_DISTANCE_COUNTS  = (40 * COUNTS_PER_CM);
    static final double  ACCELERATING_DISTANCE_COUNTS  = (20 * COUNTS_PER_CM);

    boolean rackup = false;
    boolean dumbdriving = false;
    int remainingdistance = 10000;

    // initializes hardware; calibrates gyro
    public void inithardware(boolean test){

        robot = new HardwareSkyStone(test);
        robot.init(hardwareMap);
        initializeObjectDetection();
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

    // moves lift down and extends slide to park in crater
    public void parkInCrater(){
        setLiftPower(-0.6);
        sleep(400);
        setLiftPower(-0.2);

        setSlidePower(1);
        sleep(1000);
        setSlidePower(0);

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


    //Object Detection functions


    public void initializeObjectDetection(){
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }
    }

   /* public String getMineralLocation(){
        String boi = "Centernotreallytho";
        runtime.reset();

        while (runtime.seconds() < 5) {
            if (tfod != null) {
                telemetry.addData("tfod","is initialized");
                telemetry.update();
                sleep(2000);
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions !=null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    telemetry.update();
                    if (updatedRecognitions.size() == 2) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getLeft();
                            } else {
                                silverMineral2X = (int) recognition.getLeft();
                            }
                        }
                        if (goldMineralX == -1) {

                            boi = "Left";
                        } else if (goldMineralX < silverMineral1X) {

                            boi = "Center";
                        } else if (goldMineralX > silverMineral1X) {

                            boi = "Right";
                        }
                        return boi;
                    }
                }

            }
        }
        return boi;
    }*/

    public float findSkyStone(){
        float skystonex = -1;
        float stone1x = -1;
        float stone2x = -1;
        while(runtime.seconds()<4){
            if (tfod != null){
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals("Skystone")){
                            skystonex = (recognition.getLeft()+recognition.getRight())/2.0f;
                        }
                    }


                    telemetry.update();
                }
            }
        }
        return skystonex;

    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

}