package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.lang.*;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        long millis=System.currentTimeMillis();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(32.5, -63, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        Trajectory traj = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(34  , -12.1, Math.toRadians(0)))
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(new Pose2d())
                .forward(7.5+11.7+23.4+7.5)
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(startPose)
                .forward(4)
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(new Pose2d())
                .forward(1.7+23.4)
                .build();
        Trajectory traj5 = drive.trajectoryBuilder(new Pose2d())
//
                .forward(39.25)
//                .addSpatialMarker(new Vector2d(30, 0),() ->{})
                .splineTo(new Vector2d(7.5+11.7+23.4+7.5+0.8, -4), Math.toRadians(-90))
                .build();
//
        waitForStart();
//clock.startmillis();
//        if (isStopRequested()) return;

//        Pose2d pose2d = new Pose2s


        drive.followTrajectory(traj);
telemetry.addData("time",System.currentTimeMillis()-millis);
telemetry.update();
//        sleep(100);
//        drive.turn(Math.toRadians(-90));
//        traj3.end();
//        sleep(100);
//        drive.setPoseEstimate(startPose);
//        drive.followTrajectory(traj5);


    }
}
