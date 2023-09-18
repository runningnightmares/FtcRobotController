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
package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.teleop.MecanumDriveHardware;

/**
     * This particular OpMode executes a POV Game style Teleop for a direct drive robot
     * The code is structured as a LinearOpMode*
     * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
     * It raises and lowers the arm using the Gamepad Y and A buttons respectively.
     * It also opens and closes the claws slowly using the left and right Bumper buttons.
     * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
     * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
     */
    @Autonomous(name="auto gyro", group="auto")
    //@Disabled
    public class MecanumAuto extends LinearOpMode {

        private final ElapsedTime runTime = new ElapsedTime();
        private DcMotor elevatorArm;
        private Servo leftClawServo;
        private IMU imu;
        private final MecanumDriveHardware robot = new MecanumDriveHardware();
        private final int WHEEL_DIAMETER = 96;// gobilda mecanum wheel Diameter 96mm or 3.78"
        private final double DRIVETRAIN_POWER = 0.5;
        private final double DRIVETRAIN_TPI = 45.33;
    private final double DRIVETRAIN_ZERO_POWER = 0.0;
        private final double ELEVATOR_ARM_POWER = 0.5;
        private final double ELEVATOR_ARM_ZERO_POWER = 0.0;

        private final double LEFT_CLAW_INIT_POSITION = 0.5;
        private final double LEFT_CLAW_POSITION_ONE = 0;
        private final double LEFT_CLAW_POSITION_TWO = 1;
        private final double ELEVATOR_ENCODER_TICKS_PER_INCH = 188.72;

        public void initHardware(){
            elevatorMotorInit();
            leftClawServoInit();
            imuInit();
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

        public void leftClawServoInit(){
            leftClawServo = hardwareMap.get(Servo.class, "leftClawServo");
            leftClawServo.setDirection(Servo.Direction.FORWARD);
            leftClawServo.setPosition(LEFT_CLAW_INIT_POSITION);
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
    @Override
        public void runOpMode(){

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
                autonomousDriveForward(0.25,10);
                motorTelemetry();

                stopAllMotors();
            }

        }


    public void runLiftArmToPosition(int position){
        elevatorArm.setTargetPosition(position);
        elevatorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorArm.setPower(ELEVATOR_ARM_POWER);
        while (elevatorArm.isBusy()){
            sleep(5);
        }
    }
    public  void resetElevatorArmEncoder(){
        //stop motor
        elevatorArm.setPower(ELEVATOR_ARM_ZERO_POWER);
        //stop and reset the encoder
        elevatorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //start the encoder
        elevatorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void stopAllMotors(){

        // Stop motors
        robot.frontLeftDrive.setPower(DRIVETRAIN_ZERO_POWER);
        robot.frontRightDrive.setPower(DRIVETRAIN_ZERO_POWER);
        robot.backLeftDrive.setPower(DRIVETRAIN_ZERO_POWER);
        robot.backRightDrive.setPower(DRIVETRAIN_ZERO_POWER);
    }
    public void motorTelemetry(){
        // Monitor telemetry and perform other tasks
        telemetry.addData("Status", "Running Autonomous");
        telemetry.addData("FL Motor:", robot.frontLeftDrive.getCurrentPosition());
        telemetry.addData("FR Motor:", robot.frontRightDrive.getCurrentPosition());
        telemetry.addData("BL Motor:", robot.backLeftDrive.getCurrentPosition());
        telemetry.addData("BR Motor:", robot.backRightDrive.getCurrentPosition());
        telemetry.update();
    }
    public void autonomousDriveForward(double power, int distanceInches) {
        int targetEncoderTicks = (int) (distanceInches * DRIVETRAIN_TPI);
        // Reset encoder values and set target positions
        robot.frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeftDrive.setTargetPosition(targetEncoderTicks);
        robot.frontRightDrive.setTargetPosition(targetEncoderTicks);
        robot.backLeftDrive.setTargetPosition(targetEncoderTicks);
        robot.backRightDrive.setTargetPosition(targetEncoderTicks);

        // Set power and run to position
        robot.frontLeftDrive.setPower(power); // Adjust power as needed
        robot.frontRightDrive.setPower(power);
        robot.backLeftDrive.setPower(power);
        robot.backRightDrive.setPower(power);

        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        telemetry.addData("Status", "Moving Forward");

        // Stop all motors
        robot.frontLeftDrive.setPower(0);
        robot.frontRightDrive.setPower(0);
        robot.backLeftDrive.setPower(0);
        robot.backRightDrive.setPower(0);

        // Set the motors back to RUN_USING_ENCODER mode for manual control
        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

}
