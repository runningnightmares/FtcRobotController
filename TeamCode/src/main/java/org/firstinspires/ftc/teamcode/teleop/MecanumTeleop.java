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

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * This particular OpMode executes a POV Game style Teleop for a direct drive robot
 * The code is structured as a LinearOpMode
 *
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the arm using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Mecanum Drive", group="Linear Opmode")
//@Disabled
public class MecanumTeleop extends LinearOpMode {

    private ElapsedTime runTime = new ElapsedTime();
    private DcMotor elevatorArm;
    private double ELEVATOR_ARM_POWER = 0.5; //lift arm power
    private double ELEVATOR_ARM_ZERO_POWER = 0.0;
    private int ELEVATOR_POSITION_ONE = 0;//ground position
    private int ELEVATOR_POSITION_TWO = 500;//low position
    private int ELEVATOR_POSITION_THREE = 1000; //mid position
    private int ELEVATOR_POSITION_FOUR = 1500; //high position
    private double ELEVATOR_ARM_SENSITIVITY = 0.5;
    private int elevatorPositionValue;
    private int currentElevatorPositionValue;
    private double leftJoyStickXAxis;
    private double leftJoyStickYAxis;
    private double rightJoyStickXAxis;
    private double rightJoyStickYAxis;

    private Servo leftClawServo;
    private double LEFT_CLAW_INIT_POSITON = 0.5;
    private double LEFT_CLAW_POSTION_ONE = 0;
    private double LEFT_CLAW_POSITION_TWO = 1;
    private double LEFT_CLAW_DELAY = 10;

    IMU imu;

    MecanumDriveHardware robot = new MecanumDriveHardware();

    public void initHardware(){
        elevatorMotorInit();
        leftClawServoInit();
        imuInit();
    }

    public void leftClawServoInit(){
        leftClawServo = hardwareMap.get(Servo.class, "leftClawServo");
        leftClawServo.setDirection(Servo.Direction.FORWARD);
        leftClawServo.setPosition(LEFT_CLAW_INIT_POSITON);
        telemetry.addData("Left Claw Position" , leftClawServo.getPosition());

    }

    public void elevatorMotorInit(){


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        elevatorArm = hardwareMap.get(DcMotor.class, "liftArm");
        elevatorArm.setDirection(DcMotorSimple.Direction.REVERSE);
        elevatorArm.setPower(ELEVATOR_ARM_POWER);
        elevatorArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //brake use motor to stop float use coasting to stop
        elevatorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    //method to run the elevator to a certain position
    public void runLiftArmToPosition(int position)
    {
        elevatorArm.setTargetPosition(position);
        elevatorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorArm.setPower(ELEVATOR_ARM_POWER);
        while (elevatorArm.isBusy()){
            sleep(5);
        }
        //liftArm.setPower(liftArmZeroPower);
    }

    public  void resetEncoder(){
        //stop motor
        elevatorArm.setPower(ELEVATOR_ARM_ZERO_POWER);
        //stop and reset the encoder
        elevatorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //start the encoder
        elevatorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void motorTelemetry() {
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runTime.toString());
        telemetry.addData("Lift Position", " Encoder: %2d, Power %2f", elevatorArm.getCurrentPosition(), elevatorArm.getPower());
    }

    public void getCurrentElevatorPosition(){
        currentElevatorPositionValue = elevatorArm.getTargetPosition();
        elevatorPositionValue = currentElevatorPositionValue;
    }
    public void teleopControls(){

        //elevator positions and controls

        rightJoyStickYAxis = -gamepad1.right_stick_y;

        if(gamepad1.dpad_right){
            runLiftArmToPosition(ELEVATOR_POSITION_ONE);
            getCurrentElevatorPosition();
        }
        if (gamepad1.dpad_down){
            runLiftArmToPosition(ELEVATOR_POSITION_TWO);
            getCurrentElevatorPosition();
        }
        if(gamepad1.dpad_left){
            runLiftArmToPosition(ELEVATOR_POSITION_THREE);
            getCurrentElevatorPosition();
        }
        if(gamepad1.dpad_up){
            runLiftArmToPosition(ELEVATOR_POSITION_FOUR);
            getCurrentElevatorPosition();
        }

        if((gamepad1.left_bumper) == true &(gamepad1.right_bumper)== true){

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

public void imuInit(){
// Retrieve the IMU from the hardware map
    imu = hardwareMap.get(IMU.class, "imu");
    // Adjust the orientation parameters to match your robot
    IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
            RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
    // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
    imu.initialize(parameters);
}
public void mecanumDrive(){
    //mecanum Drivetrain code
    // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
    // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
    // This way it's also easy to just drive straight, or just turn.

    leftJoyStickXAxis  =  gamepad1.left_stick_x * 1.1; //1.1 use to counteract imperfect strafing
    leftJoyStickYAxis = -gamepad1.left_stick_y; //y stick value is reversed
    rightJoyStickXAxis  =  gamepad1.right_stick_x;

    // This button choice was made so that it is hard to hit on accident, reset robot field position
    // reset orientation
    if (gamepad1.back) {
        imu.resetYaw();
    }

    double botOrientation = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

    // Rotate the movement direction counter to the bot's rotation
    double rotateX = leftJoyStickXAxis * Math.cos(-botOrientation) - leftJoyStickYAxis * Math.sin(-botOrientation);
    double rotateY = leftJoyStickXAxis * Math.sin(-botOrientation) + leftJoyStickYAxis * Math.cos(-botOrientation);

    rotateX = rotateX * 1.1;  // Counteract imperfect strafing
    // Denominator is the largest motor power (absolute value) or 1
    // This ensures all the powers maintain the same ratio,
    // but only if at least one is out of the range [-1, 1]
    double denominator = Math.max(Math.abs(leftJoyStickYAxis) + Math.abs(leftJoyStickXAxis) + Math.abs(rightJoyStickXAxis), 1);
    double frontLeftMotorPower = (rotateY + rotateX + rightJoyStickXAxis) / denominator;
    double backLeftMotorPower = (rotateY - rotateX + rightJoyStickXAxis) / denominator;
    double frontRightMotorPower = (rotateY - rotateX - rightJoyStickXAxis) / denominator;
    double backRightMotorPower = (rotateY + rotateX - rightJoyStickXAxis) / denominator;

    //Set motor power
    robot.frontLeftDrive.setPower(frontLeftMotorPower / 2);
    robot.backRightDrive.setPower(backRightMotorPower / 2);
    robot.frontRightDrive.setPower(frontRightMotorPower / 2);
    robot.backLeftDrive.setPower(backLeftMotorPower / 2);
}

    @Override
    public void runOpMode() {

        initHardware();
        runTime.reset();
        // Define and Initialize Motors
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");    //
        telemetry.update();




        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if (isStopRequested()) return;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            teleopControls();
            mecanumDrive();
            motorTelemetry();
            // Send telemetry message to signify robot running;
            telemetry.addData("left x1",  "%.2f", leftJoyStickXAxis);
            telemetry.addData("left y1", "%.2f", leftJoyStickYAxis);
            telemetry.addData("right x1",  "%.2f", rightJoyStickXAxis);
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);


        }




    }
}
