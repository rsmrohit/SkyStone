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

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.io.File;

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

    boolean pastNani;
    boolean pastOof;
    boolean pastBruh;
    boolean pastXp;

    boolean spinX;
    boolean spinY;

    boolean startTheDrop;


    Integer gyroAngle;



    MecanumDriveTrain vroom;


    private ElapsedTime runtime = new ElapsedTime();





    /* Declare OpMode members. */
    HardwareSkyStone robot       = new HardwareSkyStone(false); // use the class created to define a RoverRuckus's hardware


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

        pastBruh = false;
        pastNani = false;
        pastOof = false;
        pastXp = false;

        gyroAngle = null;

        spinX = false;
        spinY = false;
        startTheDrop = false;


        vroom = new MecanumDriveTrain(robot, gamepad1,telemetry);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Haddi", "Haddi");
        telemetry.update();
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



        //Mecanum Drivetrain function to set powers
        vroom.loop();

        if (gamepad2.left_bumper){
            telemetry.addData("bruh","bruh");
            robot.rightclaw.setPower(-1.0);
            robot.leftclaw.setPower(-1.0);
        }else if(gamepad2.right_bumper){
            robot.rightclaw.setPower(1.0);
            robot.leftclaw.setPower(1.0);
        } else{
            robot.rightclaw.setPower(0);
            robot.leftclaw.setPower(0);
        }


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


        telemetry.update();

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}