package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
//import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import java.lang.*;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.Encoder;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class AutoTest extends LinearOpMode {
    public Servo intakeTurn=null;
    public Servo intake=null;
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx leftFront, leftRear, rightRear, rightFront;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        leftFront = hardwareMap.get(DcMotorEx.class, "FL");
        leftRear = hardwareMap.get(DcMotorEx.class, "BL");
        rightRear = hardwareMap.get(DcMotorEx.class, "BR");
        rightFront = hardwareMap.get(DcMotorEx.class, "FR");
        DcMotor misumi=hardwareMap.dcMotor.get("misumi");
        DcMotor intakeExt=hardwareMap.dcMotor.get("intakeExt");
        DcMotor viper=hardwareMap.dcMotor.get("viper");
        DcMotor adjust=hardwareMap.dcMotor.get("adjust");
//        Servo intake=hardwareMap.get(Servo.class, "intake");

        intakeTurn = hardwareMap.get(Servo.class, "intakeTurn");
        CRServo turntable= hardwareMap.crservo.get("turntable");
        intake= hardwareMap.servo.get("intake");
        Encoder misumiEnc= new Encoder(hardwareMap.get(DcMotorEx.class, "misumi"));
        Encoder intakeExtEnc= new Encoder(hardwareMap.get(DcMotorEx.class, "intakeExt"));
        Encoder viperEnc= new Encoder(hardwareMap.get(DcMotorEx.class, "viper"));
        Pose2d startPose = new Pose2d(32.5, -63, Math.toRadians(90));
        int misval=misumiEnc.getCurrentPosition();
        misumi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        DriveConstants drive2 =new DriveConstants();
        drive.setPoseEstimate(startPose);

        Trajectory traj = drive.trajectoryBuilder(startPose)

                .lineToSplineHeading(
                        new Pose2d(36.8  , -12.4, Math.toRadians(0)))


                .build();

//.splineToSplineHeading(new Pose2d(12,-36, Math.toRadians(0)), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(maxVel, maxAngularVel, trackWidth))


//
        waitForStart();
        long millis=System.currentTimeMillis();
//clock.startmillis();
//        if (isStopRequested()) return;

//        Pose2d pose2d = new Pose2s

//        .setConstraints(16, 16, Math.toDegrees(13.369015276637452), Math.toDegrees(13.369015276637452), 7.62);
        drive.followTrajectory(traj);
        telemetry.addData("time",System.currentTimeMillis()-millis);
        telemetry.update();
        sleep(2000);
//        sleep(100);
//        drive.turn(Math.toRadians(-90));
//        traj3.end();
//        sleep(100);
//        drive.setPoseEstimate(startPose);
//        drive.followTrajectory(traj5);


    }
}
