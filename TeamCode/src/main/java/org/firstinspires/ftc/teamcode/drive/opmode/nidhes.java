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
public class nidhes extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        telemetry.addData("Started","yes");
        telemetry.update();

        if (isStopRequested()) return;

        telemetry.addData("Passed IsStopRequested","yes");
        telemetry.update();

        TrajectorySequence untitled0 = drive.trajectorySequenceBuilder(new Pose2d(-66.29, 12.04, Math.toRadians(0.00)))
            .splineTo(new Vector2d(-24.89, 11.69), Math.toRadians(-0))

            .splineTo(new Vector2d(-53.80, 11.51), Math.toRadians(0))
            .splineTo(new Vector2d(-57.01, 57.01), Math.toRadians(94.04))
            .build();
        telemetry.addData("Is built","Built");
        telemetry.update();
        drive.followTrajectorySequence(untitled0);
        telemetry.addData("Finished","Yes");
        telemetry.update();

    }
}
