package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="!Blue Front 6-7 (ENCODERS V3)", group="Autonomous")
public class BlueAuto6EncodersRev3 extends LinearOpMode {

    private Robot robot;

    // distance estimates (adjust after real testing)
    private static final double FLYWHEEL_POWER = 0.75;
    private static final double STRAFE_1 = -25;
    private static final double TURN_1 = 4;
    private static final double TURN_2 = 2;
    private static final double FWD_INTAKE = 34;
    private static final double BACK_UP = -35;
    private static final double STRAFE_BACK = 14;

    private static final double SPEED = 0.85;

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
        plan.add(this::spinUp);
        plan.add(() -> robot.updateFrontIntakeMotors(1));

        // === BACK UP ===
        plan.add(() -> robot.updateFlywheelMotors(-0.95));
        plan.add(() -> robot.driveForwardEncoder(-35, SPEED));
        plan.add(() -> robot.driveForwardEncoder(0.1,SPEED));
        plan.add(() -> robot.updateFlywheelMotors(-0.82));
        plan.add(() -> sleep(500));

        // === SHOOT 3 BALLS ===
        plan.add(this::shootOne);
        plan.add(this::loadOne);
        plan.add(this::shootOne);
        plan.add(this::loadOne);
        plan.add(this::shootOne);


        // === TURN RIGHT ===
        plan.add(() -> robot.driveForwardEncoder(-10,SPEED));
        plan.add(() -> robot.turnEncoder(-TURN_1, SPEED));



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
        plan.add(() -> robot.turnEncoder(TURN_1,SPEED));
        plan.add(() -> robot.driveForwardEncoder(9,SPEED));

        // === SHOOT 2nd ===
        plan.add(this::shootOne);
        plan.add(this::loadOne);
        plan.add(this::shootOne);
        plan.add(this::loadOne);
        plan.add(this::shootOne);
        // move to 3rd
        plan.add(() -> robot.feedStop());
        plan.add(() -> robot.turnEncoder(-3,SPEED));
        plan.add(() -> robot.strafeEncoder(-32,SPEED));
        plan.add(() -> robot.turnEncoder(1.5,SPEED));
        // 3rd set
        plan.add(() -> robot.updateIntakeMotors(1));
        plan.add(() -> robot.driveForwardEncoder(35,0.75));
        plan.add(() -> sleep(900));
        plan.add(() -> robot.updateIntakeMotors(0));
        plan.add(() -> robot.driveForwardEncoder(-32,SPEED));
        // pos to shoot
        //plan.add(() -> robot.turnEncoder(2,SPEED));
        plan.add(() -> robot.strafeEncoder(32,SPEED));
        plan.add(() -> robot.turnEncoder(2,SPEED));
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

    private void spinUp() {
        robot.updateFlywheelMotors(-1);
        sleep(2700);
    }
}
