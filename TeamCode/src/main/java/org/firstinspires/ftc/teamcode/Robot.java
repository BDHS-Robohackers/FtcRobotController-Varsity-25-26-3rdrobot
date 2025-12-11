package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {

    private static final String TAG = "Robot";
    public static final float FLY_WHEEL_MAX_THRESHOLD = 0.8f;

    public DcMotor leftFrontDrive;
    public DcMotor leftBackDrive;
    public DcMotor rightFrontDrive;
    public DcMotor rightBackDrive;
    public DcMotorEx fly;
    public DcMotor feedFly;
    public DcMotor intake;

    public Robot() {}

    // --- Encoder Constants ---
    private static final double TICKS_PER_REV = 560;       // goBILDA 20:1
    private static final double WHEEL_DIAMETER_INCHES = 3.77953; // 96mm wheels
    private static final double TICKS_PER_INCH =
            TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_INCHES);

    // --- Flywheel PIDF + smoothing ---
    private double targetRPM = 0;
    private double filteredVelocity = 0;
    private long lastUpdateTime = 0;

    private static final double RAMP_RATE = 0.015; // how fast power can change per loop
    private static final double VELOCITY_SMOOTHING = 0.7; // higher = smoother
    private static final double TICKS_PER_REV_FLY = 28; // REV 6000 motor internal encoder

    // PIDF coefficients (starter values)
    private static final double kP = 0.0020;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kF = 0.085;

    private double flywheelPower = 0;


    // -------------------------------
    // 🔧 INITIALIZATION
    // -------------------------------
    public void initialize(HardwareMap hardwareMap) {
        try {
            leftFrontDrive  = hardwareMap.get(DcMotor.class, "front_left_drive");
            leftBackDrive   = hardwareMap.get(DcMotor.class, "back_left_drive");
            rightFrontDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
            rightBackDrive  = hardwareMap.get(DcMotor.class, "back_right_drive");
            fly             = hardwareMap.get(DcMotorEx.class, "fly");
            intake          = hardwareMap.get(DcMotor.class, "intake");
            feedFly         = hardwareMap.get(DcMotor.class, "feedFly");

            // -------------------------------
            // 🔄 MOTOR DIRECTIONS
            // -------------------------------
            leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
            rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

            fly.setDirection(DcMotor.Direction.REVERSE);
            intake.setDirection(DcMotor.Direction.FORWARD);
            feedFly.setDirection(DcMotor.Direction.FORWARD);

            // -------------------------------
            // 🔧 ENCODER SETUP
            // -------------------------------
            resetDriveEncoders();

            fly.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            fly.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            feedFly.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // -------------------------------
            // 🛑 ZERO POWER BEHAVIORS
            // -------------------------------
            leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            feedFly.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            fly.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        } catch (Exception e) {
            Log.e(TAG, "Error initializing hardware", e);
        }
    }

    // -------------------------------
    // 🚗 DRIVE HELPERS WITH ENCODERS
    // -------------------------------
    public void resetDriveEncoders() {
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void runToPosMode() {
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void driveForwardEncoder(double inches, double speed) {
        resetDriveEncoders();
        int move = (int) (inches * TICKS_PER_INCH);

        leftFrontDrive.setTargetPosition(move);
        rightFrontDrive.setTargetPosition(move);
        leftBackDrive.setTargetPosition(move);
        rightBackDrive.setTargetPosition(move);

        runToPosMode();

        leftFrontDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightBackDrive.setPower(speed);

        while (leftFrontDrive.isBusy() || rightFrontDrive.isBusy() ||
                leftBackDrive.isBusy() || rightBackDrive.isBusy()) {}

        stopDriving();
    }

    public void strafeEncoder(double inches, double speed) {
        resetDriveEncoders();
        int move = (int) (inches * TICKS_PER_INCH);

        leftFrontDrive.setTargetPosition(-move);
        rightFrontDrive.setTargetPosition(move);
        leftBackDrive.setTargetPosition(move);
        rightBackDrive.setTargetPosition(-move);

        runToPosMode();

        leftFrontDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightBackDrive.setPower(speed);

        while (leftFrontDrive.isBusy() || rightFrontDrive.isBusy() ||
                leftBackDrive.isBusy() || rightBackDrive.isBusy()) {}

        stopDriving();
    }

    public void turnEncoder(double inches, double speed) {
        resetDriveEncoders();
        int move = (int) (inches * TICKS_PER_INCH);

        leftFrontDrive.setTargetPosition(move);
        leftBackDrive.setTargetPosition(move);
        rightFrontDrive.setTargetPosition(-move);
        rightBackDrive.setTargetPosition(-move);

        runToPosMode();

        leftFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        rightBackDrive.setPower(speed);

        while (leftFrontDrive.isBusy() || rightFrontDrive.isBusy() ||
                leftBackDrive.isBusy() || rightBackDrive.isBusy()) {}

        stopDriving();
    }

    public void stopDriving() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // -------------------------------
    // 🛞 FLYWHEEL + FEEDER + INTAKE
    // -------------------------------
    public void updateDriveMotors(double axial, double lateral, double yaw) {
        double max;

        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = -axial - lateral + yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = -axial + lateral + yaw;

        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    public void updateFlyFeedMotor(double power) { feedFly.setPower(power); }

    // -------------------------------
// 🎯 CLOSED-LOOP FLYWHEEL CONTROL
// -------------------------------

    public void setFlywheelRPM(double rpm) {
        targetRPM = rpm;
    }

    public void stopFlywheel() {
        targetRPM = 0;
        fly.setPower(0);
    }

    // call this once per loop (teleop)
    public void updateFlywheelControl() {
        long now = System.nanoTime();
        if (lastUpdateTime == 0) {
            lastUpdateTime = now;
            return;
        }

        double dt = (now - lastUpdateTime) / 1e9; // convert ns → seconds
        lastUpdateTime = now;

        // get raw encoder velocity (ticks per second)
        double rawVel = fly.getVelocity(); // works in RUN_USING_ENCODER

        // smooth it so it doesn't spaz at high RPM
        filteredVelocity = VELOCITY_SMOOTHING * filteredVelocity +
                (1 - VELOCITY_SMOOTHING) * rawVel;

        // convert ticks/sec → RPM
        double currentRPM = (filteredVelocity / TICKS_PER_REV_FLY) * 60.0;

        // PIDF control
        double error = targetRPM - currentRPM;
        double output = (error * kP) + (kF * (targetRPM / 6000.0)); // /6000 normalizes

        // ramp limit so power doesn’t spike & overshoot
        double maxChange = RAMP_RATE;
        output = clamp(output, flywheelPower - maxChange, flywheelPower + maxChange);

        flywheelPower = clamp(output, 0, 1);

        fly.setPower(flywheelPower);
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }


    public void updateIntakeMotors(double power) { intake.setPower(power); }
    // -------------------------------
// 🤖 INTAKE HELPERS
// -------------------------------
    public void intakeStart() {
        intake.setPower(1);
    }

    public void intakeStop() {
        intake.setPower(0);
    }

    // -------------------------------
// 📤 FEEDER HELPERS
// -------------------------------
    public void feedStart() {
        feedFly.setPower(1);
    }

    public void feedStop() {
        feedFly.setPower(0);
    }


    public double getTargetRPM() {
        return targetRPM;
    }

    public double getCurrentRPM() {
        // convert filteredVelocity → RPM
        return (filteredVelocity / TICKS_PER_REV_FLY) * 60.0;
    }

}
