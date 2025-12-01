package org.firstinspires.ftc.teamcode;

import android.util.Log; // Import for logging

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {

    private static final String TAG = "Robot"; // Tag for logging
    public static final float FLY_WHEEL_MAX_THRESHOLD = 0.7f;

    public DcMotor leftFrontDrive;
    public DcMotor leftBackDrive;
    public DcMotor rightFrontDrive;
    public DcMotor rightBackDrive;
    public DcMotor fly;
    public DcMotor feedFly;
    public DcMotor intake;

    public Robot() {}

    public void initialize(HardwareMap hardwareMap) {
        try {
            leftFrontDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
            leftBackDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
            rightFrontDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
            rightBackDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
            fly = hardwareMap.get(DcMotor.class, "fly");
            intake = hardwareMap.get(DcMotor.class, "intake");
            feedFly = hardwareMap.get(DcMotor.class, "feedFly");
        } catch (Exception e) {
            Log.e(TAG, "Error initializing hardware", e); // Replaces e.printStackTrace()
        }
    }

    /**
     * Update the drive motors with axial, lateral, yaw and an additional control (set to 0 by default for now)
     * @param axial forward is positive, backward is negative
     * @param lateral left is positive, right is negative
     * @param yaw clockwise is positive, counter-clockwise is negative
     */
    public void updateDriveMotors(double axial, double lateral, double yaw) {
        double max;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        double leftFrontPower = axial - lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial + lateral + yaw;
        double rightBackPower = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
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

    public void updateFlyFeedMotor(double power) {
        feedFly.setPower(1 * power);
    }

    public void updateFlywheelMotors(double power) {
        fly.setPower(1 * power * FLY_WHEEL_MAX_THRESHOLD);
    }

    public void updateFlywheelMotorsOverrideMax(double power) {
        fly.setPower(1 * power);
    }
    public void updateIntakeMotors(double power) {
        intake.setPower(1 * power);
    }
}

// rob was here