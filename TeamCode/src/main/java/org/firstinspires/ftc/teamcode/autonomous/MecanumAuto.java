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
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.R;
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
        private BNO055IMU imu;
        private final MecanumDriveHardware robot = new MecanumDriveHardware();
    private Servo leftClawServo;
    private Servo rightClawServo;
        private final double WHEEL_DIAMETER = 3.78;// gobilda mecanum wheel Diameter 96mm or 3.78"
    private final double COUNTS_PER_REV = 537.7; // Replace with your motor's encoder counts per revolution
    private final double DRIVE_SPEED = 0.5; // Adjust to your desired speed
    private final double DRIVETRAIN_POWER = 0.5;
        private final double DRIVETRAIN_TPI = 45.33;
    private final double DRIVETRAIN_ZERO_POWER = 0.0;
        private final double ELEVATOR_ARM_POWER = 0.5;
        private final double ELEVATOR_ARM_ZERO_POWER = 0.0;
    private final double LEFT_CLAW_INIT_POSITION = 0;
    private final double RIGHT_CLAW_INIT_POSITION = 0;
    private final double LEFT_CLAW_POSITION_ONE = 0.0;
    private final double LEFT_CLAW_POSITION_TWO = 0.45;
    private final double RIGHT_CLAW_POSITION_ONE = 0.0;
    private final double RIGHT_CLAW_POSITION_TWO = 0.45;
        private final double ELEVATOR_ENCODER_TICKS_PER_INCH = 188.72;
        private final int targetEncoderTicks = 0;

    // Set a target heading (e.g., 0 degrees for straight forward)
    double targetHeading = 0.0;

    public void initHardware(){
            elevatorMotorInit();
            leftClawServoInit();
            rightClawServoInit();
            imuInit();
        }
        public void imuInit(){

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        }

    public void leftClawServoInit(){
        leftClawServo = hardwareMap.get(Servo.class, "leftClawServo");
        leftClawServo.setDirection(Servo.Direction.FORWARD);
        leftClawServo.setPosition(LEFT_CLAW_INIT_POSITION);
    }
    public void rightClawServoInit(){
        rightClawServo = hardwareMap.get(Servo.class, "rightClawServo");
        rightClawServo.setDirection(Servo.Direction.REVERSE);
        rightClawServo.setPosition(RIGHT_CLAW_INIT_POSITION);
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
        double initialPower = 0.1; // Start with a low power
        double targetPower = 0.5; // Your desired power
        double powerIncrement = 0.05; // Increment in power

            // Wait for the game to start (driver presses PLAY)
            waitForStart();

            if (isStopRequested()) return;
            closeCLaw();
            sleep(1000);
        motorTelemetry();
        autonomousDriveStraight(.5,5,false);
        motorTelemetry();
        rotate(0.3,45);
        autonomousDriveStraight(.5,5,true);
        motorTelemetry();
        rotate(0.3,-45);
        motorTelemetry();
        strafeRight(0.5,6);
        motorTelemetry();
        autonomousDriveStraight(0.5,5,false);
        motorTelemetry();
        strafeRight(0.5,5);
        motorTelemetry();
        strafeLeft(0.5 , 10);
        motorTelemetry();
            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {



                stopAllMotors();
            }

        }


    public void runLiftArmToPosition(int position){
        elevatorArm.setTargetPosition(position);
        elevatorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorArm.setPower(ELEVATOR_ARM_POWER);
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
        // Add telemetry for debugging
        telemetry.addData("FL Position", robot.frontLeftDrive.getCurrentPosition());
        telemetry.addData("FR Position", robot.frontRightDrive.getCurrentPosition());
        telemetry.addData("BL Position", robot.backLeftDrive.getCurrentPosition());
        telemetry.addData("BR Position", robot.backRightDrive.getCurrentPosition());
        telemetry.update();
    }
    public void autonomousDriveStraight(double maxPower, int distanceInches, boolean reverse) {
        int targetPosition;

        if (reverse) {
            targetPosition = (int) (-distanceInches * COUNTS_PER_REV / (Math.PI * WHEEL_DIAMETER));
        } else {
            targetPosition = (int) (distanceInches * COUNTS_PER_REV / (Math.PI * WHEEL_DIAMETER));
        }

        // Reset encoder values and set target positions
        robot.frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeftDrive.setTargetPosition(targetPosition);
        robot.frontRightDrive.setTargetPosition(targetPosition);
        robot.backLeftDrive.setTargetPosition(targetPosition);
        robot.backRightDrive.setTargetPosition(targetPosition);
        // Initialize power and run to position
        double power = 0.1; // Initial power (start slow)
        double accelerationRate = 0.02; // Rate of power increase per iteration


        // Set power and run to position
        robot.frontLeftDrive.setPower(power); // Adjust power as needed
        robot.frontRightDrive.setPower(power);
        robot.backLeftDrive.setPower(power);
        robot.backRightDrive.setPower(power);

        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double initialHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;


        while (opModeIsActive() &&
                robot.frontLeftDrive.isBusy() &&
                robot.frontRightDrive.isBusy() &&
                robot.backLeftDrive.isBusy() &&
                robot.backRightDrive.isBusy()) {
            // Gradually increase power for acceleration
            if (power < maxPower) {
                power += accelerationRate;
            }
            // You can add additional tasks here if needed
            // Calculate the correction based on the current heading
            double currentHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            double correctionFrontLeft = (currentHeading - initialHeading) * 0.01; // Adjust the correction factor as needed
            double correctionFrontRight = (currentHeading - initialHeading) * 0.01; // Adjust the correction factor as needed
            double correctionBackLeft = (currentHeading - initialHeading) * 0.01; // Adjust the correction factor as needed
            double correctionBackRight = (currentHeading - initialHeading) * 0.01; // Adjust the correction factor as needed

            // Apply the correction to maintain a straight heading
            robot.frontLeftDrive.setPower(power + correctionFrontLeft);
            robot.frontRightDrive.setPower(power + correctionFrontRight);
            robot.backLeftDrive.setPower(power + correctionBackLeft);
            robot.backRightDrive.setPower(power + correctionBackRight);
        }

        // Gradually decrease power for deceleration
        while (power > 0) {
            power -= accelerationRate;
            if (power < 0) {
                power = 0;
            }
            robot.frontLeftDrive.setPower(power);
            robot.frontRightDrive.setPower(power);
            robot.backLeftDrive.setPower(power);
            robot.backRightDrive.setPower(power);
        }
        // Stop the robot
        robot.frontLeftDrive.setPower(0.0);
        robot.frontRightDrive.setPower(0.0);
        robot.backLeftDrive.setPower(0.0);
        robot.backRightDrive.setPower(0.0);
        // Set the motors back to RUN_USING_ENCODER mode for manual control
        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void strafeRight(double maxPower, int distanceInches) {
        int targetPosition = (int) (distanceInches * COUNTS_PER_REV / (Math.PI * WHEEL_DIAMETER));

        // Reset encoder values and set target positions
        robot.frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeftDrive.setTargetPosition(targetPosition);
        robot.frontRightDrive.setTargetPosition(-targetPosition);
        robot.backLeftDrive.setTargetPosition(-targetPosition);
        robot.backRightDrive.setTargetPosition(targetPosition);

        // Initialize power and run to position
        double power = 0.1; // Initial power (start slow)
        double accelerationRate = 0.02; // Rate of power increase per iteration

        // Set power and run to position
        robot.frontLeftDrive.setPower(power); // Adjust power as needed
        robot.frontRightDrive.setPower(-power);
        robot.backLeftDrive.setPower(-power);
        robot.backRightDrive.setPower(power);

        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() &&
                robot.frontLeftDrive.isBusy() &&
                robot.frontRightDrive.isBusy() &&
                robot.backLeftDrive.isBusy() &&
                robot.backRightDrive.isBusy()) {
            // Gradually increase power for acceleration
            if (power < maxPower) {
                power += accelerationRate;
            }
            // You can add additional tasks here if needed
        }

        // Gradually decrease power for deceleration
        while (power > 0) {
            power -= accelerationRate;
            if (power < 0) {
                power = 0;
            }
            robot.frontLeftDrive.setPower(power);
            robot.frontRightDrive.setPower(-power);
            robot.backLeftDrive.setPower(-power);
            robot.backRightDrive.setPower(power);
        }

        // Stop the robot
        stopAllMotors();

        // Set the motors back to RUN_USING_ENCODER mode for manual control
        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void strafeLeft(double maxPower, int distanceInches) {
        int targetPosition = (int) (-distanceInches * COUNTS_PER_REV / (Math.PI * WHEEL_DIAMETER));

        // Reset encoder values and set target positions
        robot.frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeftDrive.setTargetPosition(targetPosition);
        robot.frontRightDrive.setTargetPosition(-targetPosition);
        robot.backLeftDrive.setTargetPosition(-targetPosition);
        robot.backRightDrive.setTargetPosition(targetPosition);

        // Initialize power and run to position
        double power = 0.1; // Initial power (start slow)
        double accelerationRate = 0.02; // Rate of power increase per iteration

        // Set power and run to position
        robot.frontLeftDrive.setPower(-power); // Adjust power as needed
        robot.frontRightDrive.setPower(power);
        robot.backLeftDrive.setPower(power);
        robot.backRightDrive.setPower(-power);

        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() &&
                robot.frontLeftDrive.isBusy() &&
                robot.frontRightDrive.isBusy() &&
                robot.backLeftDrive.isBusy() &&
                robot.backRightDrive.isBusy()) {
            // Gradually increase power for acceleration
            if (power < maxPower) {
                power += accelerationRate;
            }
            // You can add additional tasks here if needed

        }

        // Gradually decrease power for deceleration
        while (power > 0) {
            power -= accelerationRate;
            if (power < 0) {
                power = 0;
            }
            robot.frontLeftDrive.setPower(-power);
            robot.frontRightDrive.setPower(power);
            robot.backLeftDrive.setPower(power);
            robot.backRightDrive.setPower(-power);

        }

        // Stop the robot
        stopAllMotors();

        // Set the motors back to RUN_USING_ENCODER mode for manual control
        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void rotate(double maxPower, double targetAngle) {
        // Ensure the target angle is within the range of -180 to 180 degrees
        targetAngle = normalizeAngle(targetAngle);

        // Set the motors to run using encoders
        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Calculate the initial heading
        double initialHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        // Calculate the target heading directly
        double targetHeading = initialHeading - targetAngle;

        // While the current heading is not close to the target heading, continue rotating
        while (opModeIsActive() && !isCloseEnough(initialHeading, targetHeading)) {
            // Recalculate the current heading at each iteration
            initialHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            // Calculate the error (difference between current and target headings)
            double error = targetHeading - initialHeading;

            // Normalize the error to the range -180 to 180 degrees
            error = normalizeAngle(error);

            // Adjust the power based on the error and maxPower
            double power = Math.min(maxPower, Math.abs(error / 30.0));

            // Determine the direction of rotation (clockwise or counter-clockwise)
            if (error < 0) {
                // Rotate clockwise
                robot.frontLeftDrive.setPower(power);
                robot.frontRightDrive.setPower(-power);
                robot.backLeftDrive.setPower(power);
                robot.backRightDrive.setPower(-power);
            } else {
                // Rotate counter-clockwise
                robot.frontLeftDrive.setPower(-power);
                robot.frontRightDrive.setPower(power);
                robot.backLeftDrive.setPower(-power);
                robot.backRightDrive.setPower(power);
            }
        }

        // Stop the motors
        stopAllMotors();

        // Set the motors back to RUN_USING_ENCODER mode for manual control
        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
public void openCLaw(){
       leftClawServo.setPosition(LEFT_CLAW_POSITION_ONE);
       rightClawServo.setPosition(RIGHT_CLAW_POSITION_ONE);
}
    public void closeCLaw(){
        leftClawServo.setPosition(LEFT_CLAW_POSITION_TWO);
        rightClawServo.setPosition(RIGHT_CLAW_POSITION_TWO);
        }
    // Helper function to check if the current heading is close enough to the target heading
    private boolean isCloseEnough(double currentHeading, double targetHeading) {
        // Define a tolerance for how close is considered "close enough"
        double tolerance = 5.0; // Adjust as needed

        // Calculate the absolute error
        double error = Math.abs(targetHeading - currentHeading);

        // Check if the error is within the tolerance
        return error <= tolerance;
    }

    // Helper function to normalize angles to the range -180 to 180 degrees
    private double normalizeAngle(double angle) {
        while (angle > 180.0) {
            angle -= 360.0;
        }
        while (angle <= -180.0) {
            angle += 360.0;
        }
        return angle;
    }


}
