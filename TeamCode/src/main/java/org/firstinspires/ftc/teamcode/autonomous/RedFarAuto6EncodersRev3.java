package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Robot;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="!Red Far 6-7 (ENCODERS V3)", group="Autonomous")
public class RedFarAuto6EncodersRev3 extends LinearOpMode {

    private Robot robot;

    // distance estimates (adjust after real testing)
    private static final double FLYWHEEL_POWER = 0.75;
    private static final double STRAFE_1 = -25;
    private static final double TURN_1 = 7;
    private static final double TURN_2 = 2;
    private static final double FWD_INTAKE = 35;
    private static final double BACK_UP = -35;
    private static final double STRAFE_BACK = 14;

    private static final double SPEED = 0.85;

    // 200 - MAX
    private final ElapsedTime intervalTime = new ElapsedTime();
    public static double TARGET_FLY_SPEED = 195; // Degrees per second
    public static double TARGET_FLY_SPEED_THRESH = 13; // Degrees per second

    public static double flyKP = 0.10; // Proportion
    public static double flyKI = 0.0011; // Integral
    public static double flyKD = 0.0009; // Derivative

    public static double flyKF = 0.001; // Feed forward; Find by taking POWER/SPEED

    public static double MAX_INTEGRAL = 5000.0; // May need to adjust as needed
    public static double MIN_INTEGRAL = -5000.0; // May need to adjust as needed

    public double flySpeedErrorPrev = 0.0;
    public double flyIntegral = 0.0;
    public double flySpeedModifier = 0.0f;

    private boolean resetPidTimer = false;

    public void flyWheelLoop() {
        if ((intervalTime.milliseconds() > 2000) || resetPidTimer) {
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

        final double output = Math.max(0, Math.min(1.0, rawOutput)); // Min changed to 0 instead of -1

        telemetry.addData("Fly Power: ", output);
        telemetry.addData("Raw Power: ", rawOutput);
        telemetry.addData("Interval: ", dt);

        robot.updateFlywheelMotorsOverrideMax(output);

        flySpeedErrorPrev = error;

        intervalTime.reset(); // Reset timer to get interval time
    }

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot();
        robot.initialize(hardwareMap);

        telemetry.addLine("AUTO READY (ENCODERS)");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Build action list
        List<Runnable> plan = new ArrayList<>();

        // Spin up flywheel
        // plan.add(this::spinUp);
        plan.add(() -> robot.updateFrontIntakeMotors(1));

        // === don't BACK UP ===
//        plan.add(() -> robot.updateFlywheelMotors(-0.95));
        //plan.add(() -> robot.driveForwardEncoder(-35, SPEED));
        //plan.add(() -> robot.driveForwardEncoder(0.5,SPEED));
//        plan.add(() -> robot.updateFlywheelMotors(-0.82));
        plan.add(() -> sleep(500));

        // === SHOOT 3 BALLS ===
        plan.add(this::shootOne);
        plan.add(this::loadOne);
        plan.add(this::shootOne);
        plan.add(this::loadOne);
        plan.add(this::shootOne);


        // === go fwd and TURN RIGHT ===
        plan.add(() -> robot.driveForwardEncoder(18,SPEED));
        plan.add(() -> robot.turnEncoder(TURN_1, SPEED));



        plan.add(() -> robot.feedStop());

        // === STRAFE + MOVE TO BALLS ===
        //plan.add(() -> robot.turnEncoder(TURN_2, SPEED)); // undo turn
        //plan.add(() -> robot.strafeEncoder(2, SPEED));
        //plan.add(() -> robot.turnEncoder(-2, SPEED)); // turn
        plan.add(() -> robot.updateIntakeMotors(1));  // intake on
        plan.add(() -> robot.driveForwardEncoder(FWD_INTAKE, 0.80)); // go forward
        plan.add(() -> sleep(650));
        plan.add(() -> robot.driveForwardEncoder(BACK_UP, SPEED));    // back up
        plan.add(() -> robot.updateIntakeMotors(0));

        // === STRAFE BACK ===
        plan.add(() -> robot.turnEncoder(-TURN_1,SPEED));
        plan.add(() -> robot.driveForwardEncoder(-18,SPEED));

        // === SHOOT 2nd ===
        plan.add(this::shootOne);
        plan.add(this::loadOne);
        plan.add(this::shootOne);
        plan.add(this::loadOne);
        plan.add(this::shootOne);
        // move to 3rd
        plan.add(() -> robot.feedStop());
        plan.add(() -> robot.turnEncoder(3,SPEED));
        plan.add(() -> robot.strafeEncoder(32,SPEED));
        plan.add(() -> robot.turnEncoder(1,SPEED));
        // 3rd set
        plan.add(() -> robot.updateIntakeMotors(1));
        plan.add(() -> robot.driveForwardEncoder(38,0.75));
        plan.add(() -> sleep(900));
        plan.add(() -> robot.updateIntakeMotors(0));
        plan.add(() -> robot.driveForwardEncoder(-32,SPEED));
        // pos to shoot
        //plan.add(() -> robot.turnEncoder(2,SPEED));
        plan.add(() -> robot.strafeEncoder(-32,SPEED));
        plan.add(() -> robot.turnEncoder(-2,SPEED));
        //plan.add(() -> robot.driveForwardEncoder(10,SPEED));
        // shoot 3rd
        plan.add(this::shootOne);
        plan.add(this::loadOne);
        plan.add(this::shootOne);
        plan.add(this::loadOne);
        plan.add(this::shootOne);

        // === shutdown ===
        plan.add(() -> {
            robot.updateFlywheelMotors(0);
            robot.updateDriveMotors(0,0,0);
            robot.updateIntakeMotors(0);
            robot.updateFlyFeedMotor(0);
        });

        // run actions sequentially while updating flywheel continuously
        for (Runnable action : plan) {
            if (!opModeIsActive()) break;
            flyWheelLoop();
            action.run();
        }

        telemetry.addLine("AUTO DONE");
        telemetry.update();
    }

    /** Shoots one ring cleanly while flywheel keeps spinning */
    private void loadOne() {
        robot.updateIntakeMotors(1);
        sleep(500);
        robot.updateIntakeMotors(0);
        sleep(100);
    }
    private void shootOne() {
        sleep(100);
        robot.updateFlyFeedMotor(1);
        sleep(250);
        robot.updateFlyFeedMotor(0);
        sleep(150);
    }

//    private void spinUp() {
//        flyWheelLoop();
//        //robot.updateFlywheelMotors(-1);
//        sleep(2700);
//    }
}
