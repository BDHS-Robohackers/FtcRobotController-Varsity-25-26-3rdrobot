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

@Autonomous(name="Red Front 6-7 (ENCODERS V3)", group="Autonomous")
public class RedAuto6EncodersRev3 extends LinearOpMode {

    private Robot robot;

    // distance estimates (adjust after real testing)
    private static final double TURN_1 = 3.5;
    private static final double FWD_INTAKE = 34;
    private static final double BACK_UP = -35;

    private static final double SPEED = 0.70;

    public static double TARGET_FLY_SPEED_THRESH = 60;

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
        plan.add(() -> sleep(100));

        // === BACK UP ===
        plan.add(() -> robot.driveForwardEncoder(-35, SPEED));
        plan.add(() -> robot.driveForwardEncoder(0.5,SPEED));
        plan.add(() -> sleep(500));

        // === SHOOT 3 BALLS ===
        plan.add(this::shootOne);
        plan.add(this::loadOne);
        plan.add(this::shootOne);
        plan.add(this::loadOne);
        plan.add(this::shootOne);


        // === TURN RIGHT ===
        plan.add(() -> robot.driveForwardEncoder(-10,SPEED));
        plan.add(() -> robot.turnEncoder(TURN_1, SPEED));



        plan.add(() -> robot.feedStop());

        // === STRAFE + MOVE TO BALLS ===
        plan.add(() -> robot.updateIntakeMotors(1));  // intake on
        plan.add(() -> robot.driveForwardEncoder(FWD_INTAKE, 0.80)); // go forward
        plan.add(() -> robot.updateIntakeMotors(0));
        plan.add(() -> sleep(650));
        plan.add(() -> robot.driveForwardEncoder(BACK_UP, SPEED));    // back up


        // === STRAFE BACK ===
        plan.add(() -> robot.turnEncoder(-TURN_1,SPEED));
        plan.add(() -> robot.driveForwardEncoder(9,SPEED));

        // === SHOOT 2nd ===
        plan.add(this::shootOne);
        plan.add(this::loadOne);
        plan.add(this::shootOne);
        plan.add(this::loadOne);
        plan.add(this::shootOne);
        // move to 3rd
        plan.add(() -> robot.feedStop());
        plan.add(() -> robot.turnEncoder(3,SPEED));
        plan.add(() -> robot.strafeEncoder(37,SPEED));
        plan.add(() -> robot.turnEncoder(1.5,SPEED));
        // 3rd set
        plan.add(() -> robot.updateIntakeMotors(1));
        plan.add(() -> robot.driveForwardEncoder(30,0.55));
        plan.add(() -> robot.updateIntakeMotors(0));
        plan.add(() -> sleep(900));
        plan.add(() -> robot.driveForwardEncoder(-32,SPEED));
        // pos to shoot
        //plan.add(() -> robot.turnEncoder(2,SPEED));
        plan.add(() -> robot.strafeEncoder(-32,SPEED));
        plan.add(() -> robot.turnEncoder(-5,SPEED));
        //plan.add(() -> robot.driveForwardEncoder(10,SPEED));
        // shoot 3rd
        plan.add(this::shootOne);
        plan.add(this::loadOne);
        plan.add(this::shootOne);
        plan.add(this::loadOne);
        plan.add(this::shootOne);
        plan.add(() -> robot.strafeEncoder(20,SPEED));
        plan.add(() -> robot.turnEncoder(8,SPEED));
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
        sleep(650);
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
        targetFlywheelVelocity = 1100;
    }
}
