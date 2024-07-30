package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class alborg extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;
        TrajectorySequence untitled1 = drive.trajectorySequenceBuilder(new Pose2d())
                .splineTo(new Vector2d(-30,-30), Math.toRadians(90))
                .build();
        TrajectorySequence untitled0 = drive.trajectorySequenceBuilder(new Pose2d(-65.75, -36.86, Math.toRadians(0.00)))
                .splineTo(new Vector2d(-37.08, -36.86), Math.toRadians(1.02))
                .splineTo(new Vector2d(-31.90, -32.77), Math.toRadians(87.32))
                .splineTo(new Vector2d(-28.67, -35.35), Math.toRadians(-2.86))
                .splineTo(new Vector2d(-31.69, -37.72), Math.toRadians(262.65))
                .splineTo(new Vector2d(-31.47, -30.18), Math.toRadians(90.00))
                .splineTo(new Vector2d(-31.04, 39.88), Math.toRadians(92.86))
                .splineTo(new Vector2d(-31.90, 54.32), Math.toRadians(101.31))
                .splineTo(new Vector2d(-9.49, 46.99), Math.toRadians(-40.82))
                .splineTo(new Vector2d(-4.31, 20.48), Math.toRadians(-88.41))
                .splineTo(new Vector2d(-3.88, -28.02), Math.toRadians(264.56))
                .splineTo(new Vector2d(-6.90, -56.26), Math.toRadians(230.44))
                .splineTo(new Vector2d(-44.62, -60.36), Math.toRadians(172.87))
                .splineTo(new Vector2d(-64.02, -45.70), Math.toRadians(142.93))
                .build();


        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), 0)
                .build();
        TrajectorySequence bettertraj = drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))
                .splineTo(new Vector2d(40,24),Math.toRadians(90))
                .splineTo(new Vector2d(0,48),Math.toRadians(180))
                .splineTo(new Vector2d(-20,24),Math.toRadians(270))
                .splineTo(new Vector2d(0,0),Math.toRadians(360))
                .build();
        drive.setPoseEstimate(new Pose2d(-24,0,0));
        drive.followTrajectorySequence(bettertraj);

        sleep(2000);

        //drive.followTrajectory(
        //        drive.trajectoryBuilder(bettertraj.end(), true)
        //                .splineTo(new Vector2d(0, 0), Math.toRadians(180))
        //                .build()
        //);
    }
}
