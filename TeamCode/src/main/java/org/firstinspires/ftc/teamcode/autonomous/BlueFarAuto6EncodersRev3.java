package org.firstinspires.ftc.teamcode.autonomous;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Robot;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="Blue Far 3-5 (ENCODERS V3)", group="Autonomous")
public class BlueFarAuto6EncodersRev3 extends LinearOpMode {

    private Robot robot;

    // distance estimates (adjust after real testing)
    private static final double FWD_INTAKE = 42;
    private static final double BACK_UP = -42;

    private static final double SPEED = 0.65;

    @SuppressWarnings("unused")
    public static double TARGET_FLY_SPEED_THRESH = 60;

    @SuppressWarnings("unused")
    public double currentFlywheelVelocity = 0;

    double P = 115;
    double F = 15;
    double targetFlywheelVelocity = 0;
    PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);


    @SuppressWarnings("RedundantThrows")
    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot();
        robot.initialize(hardwareMap);

        telemetry.addLine("AUTO READY (ENCODERS)");
        telemetry.update();
        robot.leftFrontDrive.setDirection(REVERSE);
        robot.fly.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);


        waitForStart();
        if (isStopRequested()) return;
        robot.fly.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.fly.setDirection(DcMotorEx.Direction.REVERSE);
        robot.updateFrontIntakeMotors(1);
        robot.leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Build action list
        List<Runnable> plan = new ArrayList<>();

        // Spin up flywheel
        plan.add(this::spinUp);
        plan.add(() -> sleep(3250));


        // === SHOOT 1st 3 BALLS ===
        plan.add(this::shootOne);
        plan.add(this::loadOne);
        plan.add(this::shootOne);
        plan.add(this::loadOne);
        plan.add(this::shootOne);


        // === Move out of shooting zone (to 2nd set) ===
        plan.add(() -> robot.driveForwardEncoder(4,SPEED));
        plan.add(() -> robot.turnEncoder(-6,SPEED));

        plan.add(() -> robot.feedStop());

        // === 2nd set ===
        plan.add(() -> robot.updateIntakeMotors(1));  // intake on
        plan.add(() -> robot.driveForwardEncoder(FWD_INTAKE, 0.60)); // go forward
        plan.add(() -> sleep(650));
        plan.add(() -> robot.updateIntakeMotors(0));
        plan.add(() -> robot.driveForwardEncoder(BACK_UP, SPEED));    // back up


        // === STRAFE BACK ===
        plan.add(() -> robot.turnEncoder(6,SPEED));
        plan.add(() -> robot.driveForwardEncoder(-4,SPEED));

        // === SHOOT 2nd ===
        plan.add(this::shootOne);
        plan.add(this::loadOne);
        plan.add(this::shootOne);
        plan.add(this::loadOne);
        plan.add(this::shootOne);
        // move out
        plan.add(() -> robot.feedStop());
        plan.add(() -> robot.driveForwardEncoder(9,SPEED));
        plan.add(() -> robot.turnEncoder(-4,SPEED));
        plan.add(() -> robot.driveForwardEncoder(9,SPEED));

        // === shutdown ===
        plan.add(() -> {
            targetFlywheelVelocity = 0;
            robot.updateDriveMotors(0,0,0);
            robot.updateIntakeMotors(0);
            robot.updateFlyFeedMotor(0);
        });

        // run actions sequentially while updating flywheel continuously
        for (Runnable action : plan) {
            if (!opModeIsActive()) break;
            robot.fly.setVelocity(-targetFlywheelVelocity);
            action.run();
        }

        telemetry.addLine("AUTO DONE");
        telemetry.update();
    }

    /** Shoots one ring cleanly while flywheel keeps spinning */
    private void loadOne() {
        robot.updateIntakeMotors(1);
        sleep(700);
        robot.updateIntakeMotors(0);
        sleep(50);
    }
    private void shootOne() {
        sleep(50);
        robot.updateFlyFeedMotor(1);
        sleep(250);
        robot.updateFlyFeedMotor(0);
        sleep(150);
    }

    private void spinUp() {
        targetFlywheelVelocity = 1490;
    }
}
