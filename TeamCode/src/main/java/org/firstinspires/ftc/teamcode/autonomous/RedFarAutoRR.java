package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="Red Far Auto (Roadrunner)", group="Autonomous")
public class RedFarAutoRR extends LinearOpMode {

    private Robot robot;

    private static final double SPEED = 0.85;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot();
        robot.initialize(hardwareMap);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;


        // Spin intake
        robot.updateFrontIntakeMotors(1);

        sleep(3500);


        // Shoot first 3
        shootOne();
        loadOne();

        shootOne();
        loadOne();

        shootOne();


        // Move forward 18
        Trajectory forward18 =
                drive.trajectoryBuilder(drive.getPoseEstimate())
                        .forward(18)
                        .build();

        drive.followTrajectory(forward18);


        // Turn right
        drive.turn(Math.toRadians(-7));


        robot.feedStop();


        // Intake forward 35
        robot.updateIntakeMotors(1);

        Trajectory intakeForward =
                drive.trajectoryBuilder(drive.getPoseEstimate())
                        .forward(35)
                        .build();

        drive.followTrajectory(intakeForward);


        sleep(650);


        Trajectory backUp =
                drive.trajectoryBuilder(drive.getPoseEstimate())
                        .back(35)
                        .build();

        drive.followTrajectory(backUp);


        robot.updateIntakeMotors(0);


        // Turn back
        drive.turn(Math.toRadians(7));


        Trajectory back18 =
                drive.trajectoryBuilder(drive.getPoseEstimate())
                        .back(18)
                        .build();

        drive.followTrajectory(back18);


        // Shoot second set
        shootOne();
        loadOne();

        shootOne();
        loadOne();

        shootOne();


        robot.feedStop();


        drive.turn(Math.toRadians(-3));


        Trajectory strafe32 =
                drive.trajectoryBuilder(drive.getPoseEstimate())
                        .strafeRight(32)
                        .build();

        drive.followTrajectory(strafe32);


        drive.turn(Math.toRadians(-1));


        robot.updateIntakeMotors(1);

        Trajectory forward38 =
                drive.trajectoryBuilder(drive.getPoseEstimate())
                        .forward(38)
                        .build();

        drive.followTrajectory(forward38);


        sleep(900);

        robot.updateIntakeMotors(0);


        Trajectory back32 =
                drive.trajectoryBuilder(drive.getPoseEstimate())
                        .back(32)
                        .build();

        drive.followTrajectory(back32);


        Trajectory strafeBack =
                drive.trajectoryBuilder(drive.getPoseEstimate())
                        .strafeLeft(32)
                        .build();

        drive.followTrajectory(strafeBack);


        drive.turn(Math.toRadians(2));


        shootOne();
        loadOne();

        shootOne();
        loadOne();

        shootOne();


        robot.updateFlywheelMotors(0);

    }



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

}