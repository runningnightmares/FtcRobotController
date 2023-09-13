package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@Disabled
//@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")

public class teleopTemplate extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException{
        //initialize hardware in a method;
        initHardware();

        //this keeps running until start is pressed
        while (!isStarted()){

        }
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //if we wanted to run something one time put it here eg, turn off camera or reset a timer

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

        }
    }

    public void initHardware() {

        //initialize hardware here to keep code cleaner and alleviate processing time/ power needed

    }
}
