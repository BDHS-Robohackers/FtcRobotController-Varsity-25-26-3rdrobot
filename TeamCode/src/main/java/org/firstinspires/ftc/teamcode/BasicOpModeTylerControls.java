package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Driver Op Mode (Tyler's Controls)", group = "Driver Op Mode")
public class BasicOpModeTylerControls extends LinearOpMode {

    /** @noinspection FieldMayBeFinal*/
    private ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime intervalTime = new ElapsedTime();
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

    public static double TARGET_FLY_SPEED = 360; // Degrees per second

    public static double flyKP = 0.01; // Proportion
    public static double flyKI = 0.0; // Integral
    public static double flyKD = 0.0; // Derivative

    public static double flyKF = 0.0001; // Feed forward; Find by taking POWER/SPEED

    public static double MAX_INTEGRAL = 5000.0; // May need to adjust as needed
    public static double MIN_INTEGRAL = -5000.0; // May need to adjust as needed

    private double flySpeedErrorPrev = 0.0;
    private double flyIntegral = 0.0;

    private boolean resetPidTimer = false;

    /**
     * Updates the fly wheel based on a simple PID loop
     */
    public void flyWheelLoop() {
        if ((intervalTime.seconds() > 2) || resetPidTimer) {
            resetPidTimer = false;
            intervalTime.reset(); // Too much time has elapsed; reset timer and wait for next loop.
            return;
        }
        final double dt = intervalTime.milliseconds() / 1000.0; // Last Interval time in seconds
        final double speed = ((DcMotorEx) robot.fly).getVelocity(AngleUnit.DEGREES);

        final double error = TARGET_FLY_SPEED - speed;

        final double feedForward = flyKF * TARGET_FLY_SPEED;

        flyIntegral += error * dt;
        flyIntegral = Math.max(MIN_INTEGRAL, Math.min(MAX_INTEGRAL, flyIntegral));

        final double derivative = (error - flySpeedErrorPrev) / dt;
        final double rawOutput = feedForward + (flyKP * error) + (flyKI * flyIntegral) + (flyKD * derivative);

        final double output = Math.max(-1.0, Math.min(1.0, rawOutput));

        robot.fly.setPower(output);

        flySpeedErrorPrev = error;

        intervalTime.reset(); // Reset timer to get interval time
    }

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
            updateDrive();
            updateFlywheel();
            updateIntake();
            telemetry.addData("Flywheel %",flywheelControl*100);
            telemetry.addData("Intake %",intakeControl*100);
            telemetry.addData("Yaw (rotate) %", yaw);
            telemetry.addData("Axial (FW/RV) %", axial);
            telemetry.addData("Lateral (Strafe) %", lateral);
            // Update Ethan servo control based on D-pad input

            telemetry.update();
        }
    }

    // Update driving controls (tank drive or similar).
    private void updateDrive() {
        axial = (1.0 * driverController.left_stick_y); // FWD/REV
        yaw = (1.0 * driverController.right_stick_x); // Rotate
        lateral = (0.6 * (driverController.left_trigger - driverController.right_trigger)); // Strafing

        robot.updateDriveMotors(axial, lateral, yaw);
    }

    // Update flywheel motors based on X and B button presses
    private void updateFlywheel() {

        // variable control based on buttons pushed
        if (otherController.right_bumper) {
            flywheelControl = 0;
            resetPidTimer = true;
        } else if (otherController.x) {
            flywheelControl = 0.75;
        } else if (otherController.b) {
            flywheelControl = 0.90;
        } else if (otherController.start) {
            if (!isTheButtonPressed) {
                flywheelControl += 0.05;
                isTheButtonPressed = true;
            }
        } else if (otherController.back) {
            if (!isTheButtonPressed) {
                flywheelControl -= 0.05;
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
            flyWheelLoop(); // Variable speed not needed anymore
//            robot.updateFlywheelMotors(flywheelControl);  // variable speed on flywheel depending on button
        } else {
            robot.updateFlyFeedMotor(0.0f);
            robot.updateFlywheelMotors(0.0f);  // Stop flywheel
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