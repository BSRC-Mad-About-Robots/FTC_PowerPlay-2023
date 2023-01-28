package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;
        double x=0;
        double y=0;
        double q=0;

//        q++;
    drive.setPoseEstimate(new Pose2d(0, 0, drive.getRawExternalHeading()));
        Trajectory traj = drive.trajectoryBuilder(new Pose2d(0,0))
                .splineTo(new Vector2d(30, 30), 0)
                .build();

        drive.followTrajectory(traj);

        sleep(4000);

    Pose2d myPose = new Pose2d(30, 30, drive.getRawExternalHeading());
    drive.setPoseEstimate(myPose);

    drive.followTrajectory(
                drive.trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(-1.9, 0.45), Math.toRadians(180))
                        .build()
        );
//

    sleep(2000);


    }
}
