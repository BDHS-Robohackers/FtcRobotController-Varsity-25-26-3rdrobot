package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Driver Op Mode (Tyler's Controls)", group = "Driver Op Mode")
public class BasicOpModeTylerControls extends LinearOpMode {

    /** @noinspection FieldMayBeFinal*/
    private ElapsedTime runtime = new ElapsedTime();
    private Robot robot;

    private Gamepad driverController;
    private Gamepad otherController;

    private double flywheelControl = 0;
    private double intakeControl = 0;
    private boolean isTheButtonPressed = false;
    private boolean isYPressed = false;

    private double axial = 0;
    private double lateral = 0;
    private double yaw = 0;

    @Override
    public void runOpMode() {
        driverController = gamepad1;
        otherController = gamepad2;
        robot = new Robot();
        robot.initialize(hardwareMap);

        // Wait for the robot to start (driver presses PLAY).
        telemetry.addData("Status", "Initialized.");
        telemetry.addData("Controls", "Use the following controls:");
        telemetry.addData("Strafe Left", "Left Trigger");
        telemetry.addData("Strafe Right", "Right Trigger");
        telemetry.addData("Forward/Back", "LEFT Stick up/down");
        telemetry.addData("Rotate", "RIGHT Stick left/right");
        telemetry.addData("Flywheel On", "Push X to Switch on");
        telemetry.addData("Flywheel Off", "Push B to Switch off");
        telemetry.addData("Ethan Servo Control", "D-pad Up: Forward, D-pad Down: Reverse");
        telemetry.addData("Good luck!", "DON'T CRASH THE ROBOT PLS :)");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP or time runs out).
        while (opModeIsActive()) {
            robot.updateFlywheelControl();
            updateDrive();
            updateFlywheel();
            updateIntake();
            telemetry.addData("Flywheel %",flywheelControl*100);
            telemetry.addData("Intake %",intakeControl*100);
            telemetry.addData("Yaw (rotate) %", yaw);
            telemetry.addData("Axial (FW/RV) %", axial);
            telemetry.addData("Lateral (Strafe) %", lateral);
            telemetry.addData("Flywheel Target RPM", robot.getTargetRPM());
            telemetry.addData("Flywheel Current RPM", robot.getCurrentRPM());


            // Update Ethan servo control based on D-pad input

            telemetry.update();
        }
    }

    // Update driving controls (tank drive or similar).
    private void updateDrive() {
        axial = (1.0 * driverController.right_stick_x); // Forward Back
        lateral = (0.6 * (driverController.left_trigger - driverController.right_trigger)); // Strafing
        yaw = (-0.6 * driverController.left_stick_y); // Rotate

        robot.updateDriveMotors(axial, lateral, yaw);
    }

    // Update flywheel motors based on X and B button presses
    private void updateFlywheel() {

        // variable control based on buttons pushed
        if (otherController.right_bumper) {
            flywheelControl = 0;
        } else if (otherController.x) {
            flywheelControl = 3600;
        } else if (otherController.b) {
            flywheelControl = 4320;
        } else if (otherController.start) {
            if (!isTheButtonPressed) {
                flywheelControl += 240;
                isTheButtonPressed = true;
            }
        } else if (otherController.back) {
            if (!isTheButtonPressed) {
                flywheelControl -= 240;
                isTheButtonPressed = true;
            }
        } else {
            isTheButtonPressed = false;
        }

        isYPressed = otherController.y;

        if (isYPressed) {
            robot.updateFlyFeedMotor(1);
        } else {
            robot.updateFlyFeedMotor(0);
        }

        if (flywheelControl != 0) {
            robot.setFlywheelRPM(flywheelControl);  // variable speed on flywheel depending on button
        } else {
            robot.updateFlyFeedMotor(0.0f);
            robot.stopFlywheel();  // Stop flywheel
        }
    }

    private void updateIntake() {
        if (otherController.dpad_up) {
            intakeControl = 1;
        } else if (otherController.dpad_down) {
            intakeControl = -1;
        } else {
            intakeControl = 0;
        }
        robot.updateIntakeMotors(intakeControl);
    }

    // Update Ethan servo based on D-pad input

}