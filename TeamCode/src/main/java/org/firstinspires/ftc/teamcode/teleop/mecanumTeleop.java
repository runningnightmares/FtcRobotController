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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
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
public class mecanumTeleop extends LinearOpMode {


    mecanumDriveHardware robot = new mecanumDriveHardware();

    @Override
    public void runOpMode() {
        double leftjoy_x1;
        double leftjoy_y1;
        double rightjoy_x1;


        // Define and Initialize Motors
      robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");    //
        telemetry.update();

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if (isStopRequested()) return;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.

            leftjoy_x1  =  gamepad1.left_stick_x * 1.1; //1.1 use to counteract imperfect strafing
            leftjoy_y1 = -gamepad1.left_stick_y; //y stick value is reversed
            rightjoy_x1  =  gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident, reset robot field position
            // reset orientation 
            if (gamepad1.back) {
                imu.resetYaw();
            }

            double botOrientation = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotateX = leftjoy_x1 * Math.cos(-botOrientation) - leftjoy_y1 * Math.sin(-botOrientation);
            double rotateY = leftjoy_x1 * Math.sin(-botOrientation) + leftjoy_y1 * Math.cos(-botOrientation);

            rotateX = rotateX * 1.1;  // Counteract imperfect strafing
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(leftjoy_y1) + Math.abs(leftjoy_x1) + Math.abs(rightjoy_x1), 1);
            double frontLeftPower = (rotateY + rotateX + rightjoy_x1) / denominator;
            double backLeftPower = (rotateY - rotateX + rightjoy_x1) / denominator;
            double frontRightPower = (rotateY - rotateX - rightjoy_x1) / denominator;
            double backRightPower = (rotateY + rotateX - rightjoy_x1) / denominator;


            //Set motor power
            robot.frontLeftDrive.setPower(frontLeftPower / 2);
            robot.backRightDrive.setPower(backRightPower / 2);
            //strafe left and right
            robot.frontRightDrive.setPower(frontRightPower / 2);
            robot.backLeftDrive.setPower(backLeftPower / 2);


            // Send telemetry message to signify robot running;
            telemetry.addData("left x1",  "%.2f", leftjoy_x1);
            telemetry.addData("left y1", "%.2f", leftjoy_y1);
            telemetry.addData("right x1",  "%.2f", rightjoy_x1);
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}
