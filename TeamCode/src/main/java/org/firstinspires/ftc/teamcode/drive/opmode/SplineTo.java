package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
//import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import java.lang.*;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.Encoder;

@Autonomous(group = "drive")
public class SplineTo extends LinearOpMode {
    public Servo intakeTurn=null;
    public Servo intake=null;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Encoder leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "FL"));
//        Encoder rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "BL"));
        Encoder frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "BL"));
        DcMotor misumi = hardwareMap.dcMotor.get("misumi");
        DcMotor intakeExt = hardwareMap.dcMotor.get("intakeExt");
        DcMotor viper = hardwareMap.dcMotor.get("viper");
        DcMotor adjust = hardwareMap.dcMotor.get("adjust");
        intakeTurn = hardwareMap.get(Servo.class, "intakeTurn");
        CRServo turntable = hardwareMap.crservo.get("turntable");
        intake = hardwareMap.servo.get("intake");
        Encoder misumiEnc = new Encoder(hardwareMap.get(DcMotorEx.class, "misumi"));
        Encoder intakeExtEnc = new Encoder(hardwareMap.get(DcMotorEx.class, "intakeExt"));
        Encoder viperEnc = new Encoder(hardwareMap.get(DcMotorEx.class, "viper"));
        Pose2d startPose = new Pose2d(32.5, -63, Math.toRadians(90));
        int misval = misumiEnc.getCurrentPosition();
        int intval = intakeExtEnc.getCurrentPosition();
        int vipval = viperEnc.getCurrentPosition();
        misumi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeExt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        misumi.setDirection(DcMotor.Direction.REVERSE);
        double n = 0.5;
        int misumiTicks = -248;
        boolean misumiYES = false;
        drive.setPoseEstimate(startPose);
        Trajectory traj = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(
                        new Pose2d(35.9, -12.8, Math.toRadians(0)))
                .build();
        telemetry.addData("init", true);
        telemetry.addData("misval", misval);
        telemetry.update();
        waitForStart();

//clock.startmillis();
//        if (isStopRequested()) return;

//        Pose2d pose2d = new Pose2s

//        .setConstraints(16, 16, Math.toDegrees(13.369015276637452), Math.toDegrees(13.369015276637452), 7.62);
//        drive.followTrajectory(traj);
//        sleep(100);
        viper.setPower(-1*n);
        misumi.setPower(0.8);
        while((intakeExtEnc.getCurrentPosition()-intval)>=-750) {
//                if(((intakeExtEnc.getCurrentPosition()-intval)<=-700)){viper.setPower(0.5);}

            telemetry.addData("int",intakeExtEnc.getCurrentPosition()-intval);


            if ((misumiEnc.getCurrentPosition() - misval) <= -100 && (intakeExtEnc.getCurrentPosition() - intval) > -360) {

                intakeExt.setPower(-0.4* n);
            }
            if((misumiEnc.getCurrentPosition()-misval)<=-360){
                intakeTurn.setPosition(0.965);
                sleep(30);
                intake.setPosition(1);
                sleep(260);
                intakeExt.setPower(-0.9 * n);
                misumiYES=true;
            }
            if((viper.getCurrentPosition()-vipval)<=-2020&&(viper.getCurrentPosition()-vipval)>=-2470){

                viper.setPower(-0.4*n);
            }
            if((viper.getCurrentPosition()-vipval)<=-2500){
                viper.setPower(0.82*n);
            }
            if((viper.getCurrentPosition()-vipval)>=0) {
                viper.setPower(0.0);
            }
            if(misumiYES){
                if(((misumiEnc.getCurrentPosition()-misval)<=-180)){
                    misumi.setPower(-0.1);
                }
                else
                    telemetry.addData("going down",false);
                misumi.setPower(0.1);
            }
            telemetry.update();
        }
        telemetry.addData("int",intakeExtEnc.getCurrentPosition()-intval);
        telemetry.update();
//        misumi.setPower(0.1);
        intakeExt.setPower(-0.00);
        intake.setPosition(1);
        sleep(10);

//        while(((misumiEnc.getCurrentPosition()-misval)<=-250)){
//            misumi.setPower(-0.1);
//        }
//        misumi.setPower(-0.1);
        if((misumiEnc.getCurrentPosition()-misval)<=-188)
        {
            misumi.setPower(-0.1);
        }
        else
        {
            misumi.setPower(0.1);

        }
        viper.setPower(-1*n);
        while((viper.getCurrentPosition()-vipval)>=-2500){
            if((viper.getCurrentPosition()-vipval)<=-1900){

                viper.setPower(-0.69*n);
            }
            if((misumiEnc.getCurrentPosition()-misval)<=-188)
            {
                misumi.setPower(-0.1);
            }
            else
            {
                misumi.setPower(0.1);

            }
//            telemetry.addData("viper", (viper.getCurrentPosition()-vipval));
//            telemetry.update();
        }
        if((misumiEnc.getCurrentPosition()-misval)<=-188)
        {
            misumi.setPower(-0.1);
        }
        else
        {
            misumi.setPower(0.1);

        }
//        misumi.setPower(0.1);
//        viper.setPower(0.0);
        sleep(10);
        while((viper.getCurrentPosition()-vipval)<0)
        {
            viper.setPower(0.9*n);
            if((misumiEnc.getCurrentPosition()-misval)<=-188){
                misumi.setPower(-0.1);
            }
            else
            {
                misumi.setPower(0.1);

            }
        }
        viper.setPower(0.0);
//        misumi.setPower(0.12);
        while((misumiEnc.getCurrentPosition()-misval)>=-200){
            misumi.setPower(0.8);
        }
        misumi.setPower(0.0);
        telemetry.addData("misumi",misumiEnc.getCurrentPosition()-misval);
        telemetry.update();
//        intake.setPosition(0.3);
//        sleep(5000);
        //START OF FOR
        for(int i=0;i<5;i++){
            if(i==4)
                misumiTicks=15;
            else{
                misumiTicks+=72;
            }
            if(i==2){
                misumiTicks+=69;
            }
            else if(i==3){
                misumiTicks+=69;

            }
            intakeExt.setPower(-0.45);
            sleep(36);
            intake.setPosition(0.23);
//            sleep(10);
            intakeExt.setPower(0.0);
            sleep(270);
            misumi.setPower(1*n);
            while((misumiEnc.getCurrentPosition()-misval)>=-535){
//                telemetry.addData("misumi",misumiEnc.getCurrentPosition()-misval);
            }
            misumi.setPower(0.1);
            intakeTurn.setPosition(0.31405);
            intakeExt.setPower(1);
            while((intakeExtEnc.getCurrentPosition() - intval) <= -2) {

            }
            intakeExt.setPower(0.0);
            sleep(38);
            misumi.setPower(-0.2);
            while((misumiEnc.getCurrentPosition()-misval)<=-535){
//                telemetry.addData("misumi",misumiEnc.getCurrentPosition()-misval);
            }
            misumi.setPower(0.1);
            sleep(620);
            intake.setPosition(1);
            sleep(250);
            viper.setPower(-1*n);
            while((viperEnc.getCurrentPosition() - vipval) > -422){}
            intakeExt.setPower(-1 * n);
            if(i==4){
                sleep(30);
                misumi.setPower(0.03);
                intakeExt.setPower(0.0);
                viper.setPower(-1*n);
                while ((viperEnc.getCurrentPosition() - vipval) > -2493){}
                viper.setPower(0.0);
                sleep(30);
                viper.setPower(0.7*n);
                while((viperEnc.getCurrentPosition() - vipval) <0){}
                viper.setPower(0.0);
                break;
            }
            misumi.setPower(0.8);
            sleep(55);
            misumi.setPower(0.1);
            intakeTurn.setPosition(0.965);
            intakeExt.setPower(0.0);
            sleep(410);
            intake.setPosition(1);
            intakeExt.setPower(-0.9* n);
            misumi.setPower(-0.3);
            while ((viperEnc.getCurrentPosition() - vipval) > -2493) {
                intake.setPosition(1);
                telemetry.addData("viper",(viperEnc.getCurrentPosition() - vipval));
                telemetry.update();

                while ((intakeExtEnc.getCurrentPosition() - intval) >= -1000) {
                    telemetry.addData("stopped",false);
                    telemetry.update();
                    if((intakeExtEnc.getCurrentPosition() - intval) <= -700){intakeExt.setPower(-0.4);}
                    if ((misumiEnc.getCurrentPosition() - misval) >= misumiTicks) {
//                        telemetry.addData("stopped",true);
                        misumi.setPower(0.1);
//                        telemetry.update();
                    }
                    if((viper.getCurrentPosition()-vipval)<=-1900){

                        viper.setPower(-0.69*n);
                    }
                }

                intakeExt.setPower(-0.0);
                if ((misumiEnc.getCurrentPosition() - misval) >= misumiTicks) {
//                    telemetry.addData("stopped",true);
                    misumi.setPower(0.1);
//                    telemetry.update();
                }
            }
            viper.setPower(0.1);
            sleep(10);viper.setPower(0.7);
            while((misumiEnc.getCurrentPosition() - misval) <= misumiTicks){
//                telemetry.addData("running",true);
//                telemetry.addData("running",(misumiEnc.getCurrentPosition() - misval));
//                telemetry.update();
            }
            misumi.setPower(0.1);
//            sleep(1000);

            sleep(10);
            while ((viperEnc.getCurrentPosition() - vipval) < 0) {
                viper.setPower(0.7 * n);
                telemetry.addData("viper",(viperEnc.getCurrentPosition() - vipval));
                telemetry.update();
            }
            viper.setPower(0.0);

            telemetry.addData("misumiTicks",misumiTicks);
            telemetry.addData("i",i);
            telemetry.update();
        }
    }}
//    }}