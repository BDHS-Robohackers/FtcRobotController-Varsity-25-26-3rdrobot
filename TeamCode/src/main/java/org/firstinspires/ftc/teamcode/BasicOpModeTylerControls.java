package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
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
    private double frontIntakeControl = 1;
    private boolean isTheButtonPressed = false;
    private boolean isYPressed = false;
    private boolean isLBPressed = false;
    private boolean isDrivingEnabled = true;

    private double axial = 0;
    private double lateral = 0;
    private double yaw = 0;

    // 200 - MAX
    public static double TARGET_FLY_SPEED = 195; // Degrees per second
    public static double CLOSE_TARGET_FLY_SPEED = 160; // Degrees per second
    public static double TARGET_FLY_SPEED_THRESH = 13; // Degrees per second

    public static double flyKP = 0.10; // Proportion
    public static double flyKI = 0.0011; // Integral
    public static double flyKD = 0.0009; // Derivative

    public static double flyKF = 0.001; // Feed forward; Find by taking POWER/SPEED

    public static double MAX_INTEGRAL = 5000.0; // May need to adjust as needed
    public static double MIN_INTEGRAL = -5000.0; // May need to adjust as needed

    public double flySpeedErrorPrev = 0.0;
    public double flyIntegral = 0.0;
    public double setFlySpeed = TARGET_FLY_SPEED;
    public double flySpeedModifier = 0.0f;

    private boolean resetPidTimer = false;
    public double flyCurrent = 0;

    /**
     * Updates the fly wheel based on a simple PID loop
     */
    public void flyWheelLoop() {
        if ((intervalTime.milliseconds() > 2000) || resetPidTimer) {
            resetPidTimer = false;
            intervalTime.reset(); // Too much time has elapsed; reset timer and wait for next loop.
            return;
        }

        final double dt = intervalTime.milliseconds() / 1000.0; // Last Interval time in seconds
        final double speed = ((DcMotorEx) robot.fly).getVelocity(AngleUnit.DEGREES);

        final double error = setFlySpeed - speed;

        final double feedForward = flyKF * setFlySpeed;

        flyIntegral += error * dt;
        flyIntegral = Math.max(MIN_INTEGRAL, Math.min(MAX_INTEGRAL, flyIntegral));

        final double derivative = (error - flySpeedErrorPrev) / dt;
        final double rawOutput = feedForward + (flyKP * error) + (flyKI * flyIntegral) + (flyKD * derivative);

        final double output = Math.max(0, Math.min(1.0, rawOutput)); // Min changed to 0 instead of -1

        telemetry.addData("Fly Power: ", output);
        telemetry.addData("Raw Power: ", rawOutput);
        telemetry.addData("Interval: ", dt);

        robot.updateFlywheelMotorsOverrideMax(output);

        flySpeedErrorPrev = error;

        intervalTime.reset(); // Reset timer to get interval time
    }

    @Override
    public void runOpMode() {
        driverController = gamepad1;
        otherController = gamepad2;
        robot = new Robot();
        robot.initialize(hardwareMap);
        robot.fly.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.fly.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

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
            telemetry.addData("Integral: ", flyIntegral);
            telemetry.addData("Fly Vel (Deg/s): ", ((DcMotorEx) robot.fly).getVelocity(AngleUnit.DEGREES));
            telemetry.addData("Error prev: ", flySpeedErrorPrev);
            telemetry.addData("flywheel current : ",flyCurrent);
            // Update Ethan servo control based on D-pad input

            telemetry.update();
        }
    }

    // Update driving controls (tank drive or similar).
    private void updateDrive() {

        isDrivingEnabled = !driverController.a;

        if (isDrivingEnabled) {
            axial = (1.0 * driverController.left_stick_y); // FWD/REV
            yaw = (1.0 * driverController.right_stick_x); // Rotate
            lateral = (0.6 * (driverController.left_trigger - driverController.right_trigger)); // Strafing
            robot.updateDriveMotors(axial, lateral, yaw);
        } else {
            axial = 0;
            lateral = 0;
            yaw = 0;
            robot.lockWheels();
        }


    }

    // Update flywheel motors based on X and B button presses
    private void updateFlywheel() {

        // variable control based on buttons pushed
        if (otherController.right_bumper) {
            flywheelControl = 0;
        } else if (otherController.x) {
            flywheelControl = 0.75;
        } else if (otherController.b) {
            flywheelControl = 0.90;
        } else if (otherController.start) {
            if (!isTheButtonPressed) {
                flySpeedModifier += 5;
                //flywheelControl += 0.05;
                isTheButtonPressed = true;
            }
        } else if (otherController.back) {
            if (!isTheButtonPressed) {
                flySpeedModifier -= 5;
                // flywheelControl -= 0.05;
                isTheButtonPressed = true;
            }
        }  else if (otherController.a) {
            if (!isTheButtonPressed) {
                flySpeedModifier = 0;
                isTheButtonPressed = true;
            }
        } else {
            isTheButtonPressed = false;
        }

        isYPressed = otherController.y;
        final double motorSpeed = ((DcMotorEx) robot.fly).getVelocity(AngleUnit.DEGREES);
        flyCurrent = ((DcMotorEx) robot.fly).getCurrent(CurrentUnit.MILLIAMPS);
        boolean canShoot = motorSpeed > (setFlySpeed - TARGET_FLY_SPEED_THRESH);

        if (isYPressed && canShoot) {
            robot.updateFlyFeedMotor(1);
        } else {
            robot.updateFlyFeedMotor(0);
        }

        if (flywheelControl != 0) {
            if (flywheelControl < 0.9) {
                setFlySpeed = CLOSE_TARGET_FLY_SPEED + flySpeedModifier;
                flyWheelLoop();
            } else {
                setFlySpeed = TARGET_FLY_SPEED + flySpeedModifier;
                flyWheelLoop(); // Variable speed not needed anymore
                // flyWheelLoop();
            }
        } else {
            resetPidTimer = true;
            robot.updateFlyFeedMotor(0.0f);
            robot.updateFlywheelMotors(0.0f);  // Stop flywheel
        }
    }

    private void updateIntake() {
        if (otherController.dpad_up) {
            intakeControl = -1;
        } else if (otherController.dpad_down) {
            intakeControl = 1;
        } else {
            intakeControl = 0;
        }

        if (otherController.left_bumper) {
            if (!isLBPressed) {
                if (frontIntakeControl == 1) {
                    frontIntakeControl = -1;
                } else {
                    frontIntakeControl = 1;
                }
                isLBPressed = true;
            }
        } else {
            isLBPressed = false;
        }

        robot.updateIntakeMotors(intakeControl);
        robot.updateFrontIntakeMotors(frontIntakeControl);
    }

    // Update Ethan servo based on D-pad input

}