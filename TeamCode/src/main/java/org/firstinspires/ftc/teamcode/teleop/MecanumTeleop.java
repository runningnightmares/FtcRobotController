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

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * This particular OpMode executes a POV Game style Teleop for a direct drive robot
 * The code is structured as a LinearOpMode*
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the arm using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Mecanum Drive", group = "Linear Opmode")
//@Disabled
public class MecanumTeleop extends LinearOpMode {

    private final ElapsedTime runTime = new ElapsedTime();
    private DcMotor elevatorArm;
    private Servo leftClawServo;
    private Servo rightClawServo;
    private CRServo servoTest;
    private IMU imu;
    private final MecanumDriveHardware robot = new MecanumDriveHardware();
    private final double ELEVATOR_ARM_POWER = 0.55;
    private boolean servoActivated = false; // Initialize a boolean variable to track the servo state
    private long lastButtonPressTime = 0;
    private TouchSensor intakeTouchSensor;
    private boolean intakeTouchSensorIsPressed = false;
    private NormalizedColorSensor intakeColorSensor;

    public void initHardware() {
        elevatorMotorInit();
        leftClawServoInit();
        rightClawServoInit();
        imuInit();
        servoTestInit();
        touchSensorInit();
        colorSensorInit();
    }

    @Override
    public void runOpMode() {

        initHardware();
        runTime.reset();
        // Define and Initialize Motors
        robot.init(hardwareMap);
        getTouchSensor();
        getColor();
        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if (isStopRequested()) return;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            teleopControls();
            mecanumDriveInit();
            motorTelemetry();
            getColor();
            telemetry.update();
        }

    }

    public void imuInit() {
        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
    }

    public void mecanumDriveInit() {
        //mecanum Drivetrain code
        // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
        // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
        // This way it's also easy to just drive straight, or just turn.

        double leftJoyStickXAxis = gamepad1.left_stick_x * 1.1; //1.1 use to counteract imperfect strafing
        double leftJoyStickYAxis = -gamepad1.left_stick_y; //y stick value is reversed
        double rightJoyStickXAxis = gamepad1.right_stick_x;
        double botOrientation = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double DRIVETRAIN_POWER = 0.5;

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
        robot.frontLeftDrive.setPower(frontLeftMotorPower * DRIVETRAIN_POWER);
        robot.backRightDrive.setPower(backRightMotorPower * DRIVETRAIN_POWER);
        robot.frontRightDrive.setPower(frontRightMotorPower * DRIVETRAIN_POWER);
        robot.backLeftDrive.setPower(backLeftMotorPower * DRIVETRAIN_POWER);


        /*
        Send telemetry message to signify robot running;
        telemetry.addData("left x1",  "%.2f", leftJoyStickXAxis);
        //telemetry.addData("left y1", "%.2f", leftJoyStickYAxis);
        telemetry.addData("right x1",  "%.2f", rightJoyStickXAxis);
        */
    }

    public void leftClawServoInit() {
        double LEFT_CLAW_INIT_POSITION = 0;
        leftClawServo = hardwareMap.get(Servo.class, "leftClawServo");
        leftClawServo.setDirection(Servo.Direction.FORWARD);
        leftClawServo.setPosition(LEFT_CLAW_INIT_POSITION);
    }

    public void rightClawServoInit() {
        double RIGHT_CLAW_INIT_POSITION = 0;
        rightClawServo = hardwareMap.get(Servo.class, "rightClawServo");
        rightClawServo.setDirection(Servo.Direction.REVERSE);
        rightClawServo.setPosition(RIGHT_CLAW_INIT_POSITION);
    }

    public void elevatorMotorInit() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        elevatorArm = hardwareMap.get(DcMotor.class, "liftArm");
        elevatorArm.setDirection(DcMotorSimple.Direction.REVERSE);
        elevatorArm.setPower(ELEVATOR_ARM_POWER);
        elevatorArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //brake use motor to stop float use coasting to stop
        elevatorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void servoTestInit() {
        servoTest = hardwareMap.get(CRServo.class, "servoTest");
        servoTest.setDirection(CRServo.Direction.FORWARD);
        servoTest.setPower(0);
    }

    public void touchSensorInit() {
        intakeTouchSensor = hardwareMap.get(RevTouchSensor.class, "intakeTouchSensor");
    }

    public void colorSensorInit() {
        intakeColorSensor = hardwareMap.get(NormalizedColorSensor.class, "intakeColorSensor");
    }

    public void teleopControls() {
        int ELEVATOR_POSITION_GROUND = 0;//ground position
        int ELEVATOR_POSITION_LOW = 760;//low position
        int ELEVATOR_POSITION_MID = 1260; //mid position
        int ELEVATOR_POSITION_HIGH = 1570; //high position
        getTouchSensor();// updates the touch sensor

        if (gamepad1.dpad_right) {
            runLiftArmToPosition(ELEVATOR_POSITION_GROUND);
        }
        if (gamepad1.dpad_down) {
            runLiftArmToPosition(ELEVATOR_POSITION_LOW);
        }
        if (gamepad1.dpad_left) {
            runLiftArmToPosition(ELEVATOR_POSITION_MID);
        }
        if (gamepad1.dpad_up) {
            runLiftArmToPosition(ELEVATOR_POSITION_HIGH);
        }
        if ((gamepad1.left_trigger == 1) & (gamepad1.right_trigger == 1)) {
            telemetry.addData("Elevator Encoder Reset", "True");
            telemetry.update();
            resetElevatorArmEncoder();
        }
        if (gamepad1.left_bumper) {
            closeClaw();
        }

        if (gamepad1.right_bumper) {  //close claw position
            openClaw();
        }
        if ((gamepad1.right_stick_y < -0.1) || (gamepad1.right_stick_y > 0.1)) {
            rightJoystickControl();
        }
        // This button choice was made so that it is hard to hit on accident, reset robot field position
        // reset orientation
        if (gamepad1.back) {
            imu.resetYaw();
        }

        if (gamepad1.x) {
            long currentTime = System.currentTimeMillis();
            // Adjust this delay as needed higher, longer the delay
            long debounceDelay = 300;

            if (currentTime - lastButtonPressTime > debounceDelay) {
                lastButtonPressTime = currentTime;
                servoActivated = !servoActivated; // Toggle the servo state
                if (servoActivated) {
                    servoTest.setPower(1); // Start the servo
                } else {
                    servoTest.setPower(0); // Stop the servo
                }
            }
        }
        if (intakeTouchSensorIsPressed) {
            servoTest.setPower(0);
        }
    }

    public void runLiftArmToPosition(int position) {
        elevatorArm.setTargetPosition(position);
        elevatorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorArm.setPower(ELEVATOR_ARM_POWER);
    }

    public void resetElevatorArmEncoder() {
        double ELEVATOR_ARM_ZERO_POWER = 0.0;
        elevatorArm.setPower(ELEVATOR_ARM_ZERO_POWER);
        elevatorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void motorTelemetry() {
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runTime);
        telemetry.addData("Elevator Position: ", " Encoder: %2d, Power %2f", elevatorArm.getCurrentPosition(), elevatorArm.getPower());
        telemetry.addData("intake touch sensor:", intakeTouchSensorIsPressed);
    }

    public void rightJoystickControl() { // Joystick Control of Elevator Arm

        double ELEVATOR_ENCODER_TICKS_PER_INCH = 188.6634;
        double ELEVATOR_ARM_JOYSTICK_SENSITIVITY = 0.3;
        int MAX_ELEVATOR_TRAVEL = 1600;
        int MIN_ELEVATOR_TRAVEL = -100;
        double rightJoyStickYAxis = -gamepad1.right_stick_y * ELEVATOR_ARM_JOYSTICK_SENSITIVITY; //Y value reversed

        // Calculate the new target position based on joystick input
        int newTargetPosition = elevatorArm.getTargetPosition() + (int) (rightJoyStickYAxis * ELEVATOR_ENCODER_TICKS_PER_INCH);

        // Apply limits to the target position to prevent exceeding 1600
        newTargetPosition = Math.min(newTargetPosition, MAX_ELEVATOR_TRAVEL);
        newTargetPosition = Math.max(newTargetPosition, MIN_ELEVATOR_TRAVEL);

        // Update the elevator arm's target position
        elevatorArm.setTargetPosition(newTargetPosition);

        // Apply the joystick input to control the elevator arm
        elevatorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Switch to RUN_TO_POSITION mode
        elevatorArm.setPower(ELEVATOR_ARM_POWER);

        //telemetry.addData("Right Y Axis:", rightJoyStickYAxis);
        telemetry.addData("Current Elevator Position", elevatorArm.getCurrentPosition());
    }

    public void openClaw() {
        double LEFT_CLAW_POSITION_ONE = 0.0;
        double RIGHT_CLAW_POSITION_ONE = 0.0;
        leftClawServo.setPosition(LEFT_CLAW_POSITION_ONE);
        rightClawServo.setPosition(RIGHT_CLAW_POSITION_ONE);
    }

    public void closeClaw() {
        double LEFT_CLAW_POSITION_TWO = 0.5;
        double RIGHT_CLAW_POSITION_TWO = 0.5;
        leftClawServo.setPosition(LEFT_CLAW_POSITION_TWO);
        rightClawServo.setPosition(RIGHT_CLAW_POSITION_TWO);
    }

    public void getTouchSensor() {
        intakeTouchSensorIsPressed = intakeTouchSensor.isPressed();
    }

    public void getColor() {
        // Read normalized color data
        NormalizedRGBA colors = intakeColorSensor.getNormalizedColors();
        float redColor = colors.red;     // Red component (0.0 - 1.0)
        float greenColor = colors.green; // Green component (0.0 - 1.0)
        float blueColor = colors.blue;   // Blue component (0.0 - 1.0)

        // Convert RGB to HSL
        float[] hsl = new float[3];
        Color.RGBToHSV((int) (redColor * 255), (int) (greenColor * 255), (int) (blueColor * 255), hsl);

        float hue = hsl[0]; // Hue value (0 - 360)
        float saturation = hsl[1]; // Saturation value (0.0 - 1.0)
        float lightness = hsl[2]; // Lightness value (0.0 - 1.0)

        // Define color detection thresholds (adjust these values as needed)
        float purpleHueThreshold = 210.0f; // Adjust for the hue of purple (in degrees)
        float yellowHueThreshold = 70.0f;  // Adjust for the hue of yellow (in degrees)
        float greenHueThreshold = 120.0f;  // Adjust for the hue of green (in degrees)

        float purpleSaturationThreshold = 0.4f; // Adjust for the saturation of purple
        float whiteSaturationThreshold = 0.01f; // Adjust for the saturation of white
        float purpleLightnessThreshold = 0.01f; // Adjust for the lightness of purple

        // Compare the HSL values to the thresholds
        boolean isPurple = Math.abs(hue - purpleHueThreshold) < 20.0f && saturation > purpleSaturationThreshold && lightness > purpleLightnessThreshold;
        boolean isYellow = Math.abs(hue - yellowHueThreshold) < 30.0f && saturation > 0.5f;
        boolean isGreen = Math.abs(hue - greenHueThreshold) < 30.0f && saturation > 0.5f;
        boolean isWhite = lightness > whiteSaturationThreshold;

        //telemetry.addData("Hue", hue);
        //telemetry.addData("Saturation", saturation);
        //telemetry.addData("Lightness", lightness);

        // Display the detected color
        colorTelemetry(isPurple, isYellow, isGreen, isWhite);
    }

    public void colorTelemetry(boolean isPurple, boolean isYellow, boolean isGreen, boolean isWhite) {
        // Perform actions based on color detection
        if (isPurple) {
            // Purple object detected
            telemetry.addData("color", "purple");
            // Perform corresponding actions
        } else if (isYellow) {
            // Yellow object detected
            telemetry.addData("color", "yellow");

            // Perform corresponding actions
        } else if (isGreen) {
            // Green object detected
            telemetry.addData("color", "green");
            // Perform corresponding actions
        } else if (isWhite) {
            // White object detected
            telemetry.addData("color", "white");
            // Perform corresponding actions
        } else {
            // No specific color detected
            telemetry.addData("color", "none");
            // Perform actions for other cases
        }

    }

}
