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
public class Bluebackstage extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;
        drive.setPoseEstimate(new Pose2d(0,0));

        TrajectorySequence untitled0 = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0.00)))
                .forward(30)
                        .build();


        TrajectorySequence untitled1 = drive.trajectorySequenceBuilder(untitled0.end())
                //.lineToSplineHeading(new Pose2d(20,30))
                .back(7)
                .splineToLinearHeading(new Pose2d(20,42,Math.toRadians(270)),Math.toRadians(0))

                                .build();







        drive.followTrajectorySequence(untitled0);
        sleep(1000);
        drive.followTrajectorySequence(untitled1);

    }
}