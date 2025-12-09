package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {

    private static final String TAG = "Robot";
    public static final float FLY_WHEEL_MAX_THRESHOLD = 0.8f;

    public DcMotor leftFrontDrive;
    public DcMotor leftBackDrive;
    public DcMotor rightFrontDrive;
    public DcMotor rightBackDrive;
    public DcMotor fly;
    public DcMotor feedFly;
    public DcMotor intake;

    public Robot() {}

    // --- Encoder Constants ---
    private static final double TICKS_PER_REV = 560;       // goBILDA 20:1
    private static final double WHEEL_DIAMETER_INCHES = 3.77953; // 96mm wheels
    private static final double TICKS_PER_INCH =
            TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_INCHES);

    // -------------------------------
    // 🔧 INITIALIZATION
    // -------------------------------
    public void initialize(HardwareMap hardwareMap) {
        try {
            leftFrontDrive  = hardwareMap.get(DcMotor.class, "front_left_drive");
            leftBackDrive   = hardwareMap.get(DcMotor.class, "back_left_drive");
            rightFrontDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
            rightBackDrive  = hardwareMap.get(DcMotor.class, "back_right_drive");
            fly             = hardwareMap.get(DcMotor.class, "fly");
            intake          = hardwareMap.get(DcMotor.class, "intake");
            feedFly         = hardwareMap.get(DcMotor.class, "feedFly");

            // -------------------------------
            // 🔄 MOTOR DIRECTIONS
            // -------------------------------
            leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
            rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

            fly.setDirection(DcMotor.Direction.FORWARD);
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

        double leftFrontPower = axial - lateral + yaw;
        double rightFrontPower = axial + lateral - yaw;
        double leftBackPower = axial + lateral + yaw;
        double rightBackPower = axial - lateral - yaw;

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

    public void updateFlywheelMotors(double power) { fly.setPower(power * FLY_WHEEL_MAX_THRESHOLD); }

    public void updateFlywheelMotorsOverrideMax(double power) { fly.setPower(power); }

    public void updateIntakeMotors(double power) { intake.setPower(power); }
}
