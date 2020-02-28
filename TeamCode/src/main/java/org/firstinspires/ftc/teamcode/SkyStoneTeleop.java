/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;

import static org.firstinspires.ftc.teamcode.HardwareSkyStone.TeleOpRunMode;


/**
 * This file provides basic Telop driving for a RoverRuckus robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common RoverRuckus hardware class to define the devices on the robot.
 * All device access is managed through the HardwareRoverRuckus class.
 *
 */

@TeleOp(name="Skystone Teleop: Mecanum", group="RoverRuckus")
public class SkyStoneTeleop extends OpMode{


    // declaring variables


    float bucketLimiter = 1f;

    boolean pastStateX;
    boolean pastStateY;
    boolean pastStateRbumper;
    boolean pastStateLbumper;

    boolean stopped;
    int stoptarget;

    boolean pastStateB;
    int bPresses;

    boolean autolifting;
    boolean horilifting;

    boolean pastNani;
    boolean pastOof;
    boolean pastBruh;
    boolean pastXp;

    boolean pastCap;

    double liftLimiter;

    boolean spinX;
    boolean spinY;
    boolean clamp;
    boolean unclamp;

    boolean startTheDrop;

    int bin;
    int liftTarget;
    int delta[] = new int[3];
    int past[] = new int[3];


    Integer gyroAngle;

    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.5 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_CM       = 6 ;     // This measurement is more exact than inches
    static final double     COUNTS_PER_CM         = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * Math.PI));


    OdometryGlobalCoordinatePosition globalPositionUpdate = null;


    MecanumDriveTrain vroom;


    private ElapsedTime runtime = new ElapsedTime();


    /* Declare OpMode members. */
    HardwareSkyStone robot       = new HardwareSkyStone(false); // use the class created to define a RoverRuckus's hardware
    final double COUNTS_PER_INCH = 307.699557;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */


        robot.init(hardwareMap);
        robot.setMode(TeleOpRunMode);

        // initializing the variables


        pastStateX = false;
        pastStateY = false;
        pastStateRbumper = false;
        pastStateLbumper = false;

        pastStateB = false;
        bPresses = 0;
        autolifting = false;
        horilifting = false;

        pastBruh = false;
        pastNani = false;
        pastOof = false;
        pastXp = false;

        gyroAngle = null;

        spinX = false;
        spinY = false;
        clamp = false;
        unclamp = false;
        startTheDrop = false;

        pastCap = false;
        bin = 0;
        liftLimiter = 1;

        stopped = true;
        stoptarget = robot.verticalSlider.getCurrentPosition();


        vroom = new MecanumDriveTrain(robot, gamepad1,telemetry);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Teleop", "Initialized");

        telemetry.update();
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(robot.verticalLeft, robot.verticalRight, robot.horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();
        for (int i =0;i<3;i++){
            past[i] = 0;
            delta[i] = 0;
        }


    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        stopLift(robot.verticalSlider);
        stopLift(robot.horizontalSlider);

        if (gamepad2.right_trigger >= 0.2) {
            liftLimiter = 0.5;
        } else {
            liftLimiter = 1;
        }


        //Mecanum Drivetrain function to set powers
        vroom.loop();

        if (gamepad1.left_bumper) {
//            telemetry.addData("bruh","bruh");
            robot.rightclaw.setPosition(0);
            robot.leftclaw.setPosition(1.0);
        } else if (gamepad1.right_bumper) {
            robot.rightclaw.setPosition(1.0);
            robot.leftclaw.setPosition(0);
        } else {
            robot.rightclaw.setPosition(0.5);
            robot.leftclaw.setPosition(0.5);
        }


        if (gamepad2.x && !pastCap) {
            if (robot.capstone.getPosition() == 0.2) {
                robot.capstone.setPosition(0.53);
            } else {
                robot.capstone.setPosition(0.2);
            }
        }
        pastCap = gamepad2.x;


        // turn on/off spinner
        if (gamepad1.x && !pastStateX) {
            spinX = !spinX;
            if (spinX) {
                spinY = false;
            }
        }
        pastStateX = gamepad1.x;

        // turn on/off spinner in opposite direction
        if (gamepad1.y && !pastStateY) {
            spinY = !spinY;
            if (spinY) {
                spinX = false;
            }
        }
        pastStateY = gamepad1.y;


        //If you press a, the spinner will stop spinning regardless of its initial state
        if (gamepad1.a) {
            spinY = false;
            spinX = false;
        }

        // spinner limiting and logic
        if (spinX) {
            robot.spinner.setPower(-1.0 * bucketLimiter);
            robot.spinner2.setPower(-1.0 * bucketLimiter);
        } else if (spinY) {
            robot.spinner.setPower(bucketLimiter);
            robot.spinner2.setPower(bucketLimiter);
        } else {
            robot.spinner.setPower(0);
            robot.spinner2.setPower(0);
        }


        //Clamper code
        //toggle
        if (gamepad2.right_bumper && !pastStateRbumper) {
            clamp = !clamp;
        }

        if (clamp) {
            robot.clamper.setPosition(0.14);
        } else if (!clamp) {
            robot.clamper.setPosition(0.03);
        }

            pastStateRbumper = gamepad2.right_bumper;

//            telemetry.addData("clamp","initiated");
//            telemetry.update();
//            robot.clamper.setPosition(0.14);
//        } else if (gamepad2.left_bumper) {
////            telemetry.addData("clamp","opened");
////            telemetry.update();
//
//            robot.clamper.setPosition(0.03);
//        }


            if (gamepad2.b && !pastStateB) {
                bPresses++;
                runtime.reset();
            }
            pastStateB = gamepad2.b;

            if (runtime.seconds() > 0.3) {
                if (bPresses != 0) {
                    startvertlift();
                }
                bPresses = 0;
            }


            if (!autolifting) {
                if (gamepad2.left_stick_y > 0.2 && robot.verticalSlider.getCurrentPosition() > -50) {

                    stopped = false;

                    robot.verticalSlider.setTargetPosition(-50);
                    robot.verticalSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.verticalSlider.setPower(Math.abs(gamepad2.left_stick_y * liftLimiter));

                } else if (gamepad2.left_stick_y < -0.2 && robot.verticalSlider.getCurrentPosition() < 15000) {
                    stopped = false;

                    robot.verticalSlider.setTargetPosition(15000);
                    robot.verticalSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.verticalSlider.setPower(Math.abs(gamepad2.left_stick_y * liftLimiter));

                } else {


                    if (!stopped) {
                        stoptarget = robot.verticalSlider.getCurrentPosition();
                    }
                    stopped = true;
                    robot.verticalSlider.setTargetPosition(stoptarget);
                    robot.verticalSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    robot.verticalSlider.setPower(0.5);


                }

            }

            if (gamepad2.dpad_left && !horilifting) {
                startLift(1.0, 0, robot.horizontalSlider);
            }

            if (!horilifting) {

                if (-gamepad2.right_stick_y > 0) {


                    if (robot.horizontalSlider.getMode().equals(DcMotor.RunMode.RUN_TO_POSITION)) {
                        robot.horizontalSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }

                    robot.horizontalSlider.setPower(-gamepad2.right_stick_y);

                } else if (-gamepad2.right_stick_y < 0 && robot.horizontalSlider.getCurrentPosition() > -150) {

                    robot.horizontalSlider.setTargetPosition(-150);
                    robot.horizontalSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.horizontalSlider.setPower(-gamepad2.right_stick_y);

                } else {

                    if (robot.horizontalSlider.getMode().equals(DcMotor.RunMode.RUN_TO_POSITION)) {
                        robot.horizontalSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }
                    robot.horizontalSlider.setPower(0);
                }
            }



            if (gamepad1.a){
                past[0] = robot.verticalLeft.getCurrentPosition();
                past[1] = robot.verticalRight.getCurrentPosition();
                past[2] = robot.horizontal.getCurrentPosition();
            }


        delta[0] = robot.verticalLeft.getCurrentPosition() - past[0];
        delta[1] = robot.verticalRight.getCurrentPosition() - past[1];


        delta[2] = robot.horizontal.getCurrentPosition() - past[2];





        telemetry.addData("X Position", globalPositionUpdate.getX() / COUNTS_PER_INCH);
        telemetry.addData("Y Position", globalPositionUpdate.getY() / COUNTS_PER_INCH);
        telemetry.addData("Orientation (Degrees)", globalPositionUpdate.getOrientation());
        telemetry.addData("Imu", robot.imu.getAngularOrientation().firstAngle);

//        telemetry.addData("vertical LEft", delta[0]);
//        telemetry.addData("vertical right", delta[1]);
//        telemetry.addData("horizontal", delta[2]);

        telemetry.update();

        }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        globalPositionUpdate.stop();
    }

    public void startvertlift(){

        switch(bPresses){
            case 1:
                startLift(1.0,400,robot.verticalSlider);

                break;
            case 2:
                startLift(1.0,1000,robot.verticalSlider);
                break;
            case 3:
                startLift(1.0,1480,robot.verticalSlider);
                break;
            case 4:
                startLift(1.0,2000,robot.verticalSlider);
                break;
        }

    }

    public void startLift(double speed, int target, DcMotor input ) {
        if (input.equals(robot.verticalSlider)){
            autolifting = true;
            stopped = false;
        }

        if (input.equals(robot.horizontalSlider)){
            horilifting = true;
        }


        input.setTargetPosition(target);
        input.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        input.setPower(speed);


    }

    public void stopLift(DcMotor input){
        if (!input.isBusy()){

            if (input.equals(robot.verticalSlider) && autolifting){
                autolifting = false;
                // Stop all motion;
                input.setPower(0);

                // Turn off RUN_TO_POSITION
                input.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            }

            if (input.equals(robot.horizontalSlider) && horilifting){
                horilifting = false;

                // Stop all motion;
                input.setPower(0);

                // Turn off RUN_TO_POSITION
                input.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }


        }
    }
}