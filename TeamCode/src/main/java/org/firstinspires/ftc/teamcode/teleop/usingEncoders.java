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

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;



/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

public class usingEncoders extends LinearOpMode {

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor liftArm;
    private final double liftArmPower = 0.5; //lift arm power

    public void initHardware(){
        elevatorMotorInit();
    }

    public void elevatorMotorInit(){


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        liftArm = hardwareMap.get(DcMotor.class, "liftArm");
        liftArm.setDirection(DcMotorSimple.Direction.REVERSE);
        liftArm.setPower(liftArmPower);
        liftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //brake use motor to stop float use coasting to stop
        liftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    //method to run the elevator to a certain position
    public void runLiftArmToPosition(int position)
    {
        liftArm.setTargetPosition(position);
        liftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftArm.setPower(liftArmPower);
        while (liftArm.isBusy()){
            motorTelemetry();
        }
        //liftArm.setPower(liftArmZeroPower);
    }

    public  void resetEncoder(){
        //stop motor
        double liftArmZeroPower = 0.0;
        liftArm.setPower(liftArmZeroPower);
        //stop and reset the encoder
        liftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //start the encoder
        liftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void motorTelemetry() {
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime);
        telemetry.addData("Lift Position", " Encoder: %2d, Power %2f", liftArm.getCurrentPosition(), liftArm.getPower());
        telemetry.update();
    }
    public void teleopControls(){


        /*
        * elevator positions and controls
        *
        * */
        double rightJoy_y = -gamepad1.right_stick_y;

        int liftPosValue;
        int currentLiftPosVal;
        if(gamepad1.dpad_right){
            //ground position
            int liftArmPosOne = 0;
            runLiftArmToPosition(liftArmPosOne);
            currentLiftPosVal = liftArm.getTargetPosition();
            liftPosValue = currentLiftPosVal;
        }
        if (gamepad1.dpad_down){
            //low position
            int liftArmPosTwo = 500;
            runLiftArmToPosition(liftArmPosTwo);
            currentLiftPosVal = liftArm.getTargetPosition();
            liftPosValue = currentLiftPosVal;
        }
        if(gamepad1.dpad_left){
            //mid position
            int liftArmPosThree = 1000;
            runLiftArmToPosition(liftArmPosThree);
            currentLiftPosVal = liftArm.getTargetPosition();
            liftPosValue = currentLiftPosVal;
        }
        if(gamepad1.dpad_up){
            //high position
            int liftArmPosFour = 1500;
            runLiftArmToPosition(liftArmPosFour);
            currentLiftPosVal = liftArm.getTargetPosition();
            liftPosValue = currentLiftPosVal;
        }

        if((gamepad1.left_bumper) & (gamepad1.right_bumper)){

            telemetry.addData("Lift Encoder Reset", "True");
            telemetry.update();
            resetEncoder();
        }
/*
        if((rightJoy_y > -1) || (rightJoy_y < 1)){
            //use right joystick to override motor
            liftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //liftArm.setPower(rightJoy_y);
            currentLiftPosVal = liftArm.getTargetPosition();
            liftPosValue = currentLiftPosVal;

        }*/


    }
    @Override
    public void runOpMode(){
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        initHardware();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();




        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            teleopControls();
            motorTelemetry();



        }


    }
}
