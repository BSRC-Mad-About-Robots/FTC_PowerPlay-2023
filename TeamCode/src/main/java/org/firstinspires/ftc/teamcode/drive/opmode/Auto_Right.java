package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
//import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import java.lang.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.skills.PowerplayDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;

@Autonomous(group = "drive")
public class Auto_Right extends LinearOpMode {
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
        Encoder turntableEnc = new Encoder(hardwareMap.get(DcMotorEx.class, "FR"));
        Encoder misumiEnc = new Encoder(hardwareMap.get(DcMotorEx.class, "misumi"));
        Encoder intakeExtEnc = new Encoder(hardwareMap.get(DcMotorEx.class, "intakeExt"));
        Encoder viperEnc = new Encoder(hardwareMap.get(DcMotorEx.class, "viper"));
        Pose2d startPose = new Pose2d(32.5, -63, Math.toRadians(90));
        int misval = misumiEnc.getCurrentPosition();
        int intval = intakeExtEnc.getCurrentPosition();
        int vipval = viperEnc.getCurrentPosition();
        int turnval = turntableEnc.getCurrentPosition();
        misumi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeExt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        misumi.setDirection(DcMotor.Direction.REVERSE);
        double n = 0.71;
        int misumiTicks = -250;
        boolean misumiYES = false;
        boolean viperYES = false;
        Trajectory trajectoryback = drive.trajectoryBuilder(new Pose2d())
                .back(26)
                .build();
        Trajectory trajectoryfor = drive.trajectoryBuilder(new Pose2d())
                .forward(24)
                .build();
        Trajectory trajectorystable=drive.trajectoryBuilder(new Pose2d())
                .back(5)
                .build();
//        drive.setPoseEstimate(startPose);
        Trajectory traj = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(
                        new Pose2d(37.9, -12.8, Math.toRadians(0)))
                .build();
        telemetry.addData("init", true);
        telemetry.addData("misval", misval);
        telemetry.update();
        PowerplayDetector rf = null;
        String result = "";
            try {
                //initialize the detector. It will run on its own thread continuously
                rf = new PowerplayDetector(this.hardwareMap, this, telemetry,
                        "LATEST FINAL SLEEVE RED MODEL.tflite", "SLEEVE labels.txt");
//                sleep(5000);
                telemetry.addData("initialised",true);
                telemetry.update();
                Thread detectThread = new Thread(rf);
                detectThread.start();
                while (opModeInInit()) {
                    result = rf.getResult();
                    telemetry.addData("DetectionTest result", result);
                    telemetry.update();
                    sleep( 500);
                }
            } catch (Exception ex) {
                telemetry.addData("Error", String.format("Unable to initialize Detector. %s", ex.getMessage()));
                telemetry.update();
                sleep(5000);
                return;
            }

//        long start = System.currentTimeMillis();
        waitForStart();
            turntable.setPower(-1);

//            viper.setPower(-0.3);
//        Object clock = null;
//        clock.startmillis();
//        if (isStopRequested()) return;

//        Pose2d pose2d = new Pose2s

//        .setConstraints(16, 16, Math.toDegrees(13.369015276637452), Math.toDegrees(13.369015276637452), 7.62);
//        drive.followTrajectory(trajectory);
        drive.setPoseEstimate(startPose);
        drive.followTrajectory(traj);

        //drive.followTrajectory(traj);
//        sleep(100);
            viper.setPower(-2*n);
            misumi.setPower(0.85);
            double vipertick=0.0;
            while((intakeExtEnc.getCurrentPosition()-intval)>=-855) {
//                if(((intakeExtEnc.getCurrentPosition()-intval)<=-700)){viper.setPower(0.5);}
//                telemetry.addData("int",intakeExtEnc.getCurrentPosition()-intval);
                telemetry.addData("viper Yes",viperYES);
                if((turntableEnc.getCurrentPosition()-turnval)>67000){turntable.setPower(0.0);}
                if ((misumiEnc.getCurrentPosition() - misval) <=-370&& (intakeExtEnc.getCurrentPosition() - intval) > -360) {
                    intakeExt.setPower(-0.4* n);
                }
                if((misumiEnc.getCurrentPosition()-misval)<=-380){
                    intakeTurn.setPosition(0.765);
                    intake.setPosition(1);
                    intakeTurn.setPosition(0.965);
                    sleep(10);
                    misumiYES=true;
                }
                if(intakeTurn.getPosition()>=0.96&&intake.getPosition()>0.99){
                    intakeExt.setPower(-0.9 * n);
                }
                if((viperEnc.getCurrentPosition()-vipval)<=-1700&&!viperYES&&(viperEnc.getCurrentPosition()-vipval)>=-2445)
                {
                    viper.setPower(-0.69*n);
                    telemetry.addData("viper1",(viperEnc.getCurrentPosition()-vipval));
                    telemetry.addData("decel",true);
                    telemetry.update();
                }
                if((viperEnc.getCurrentPosition()-vipval)<=-2425)
                {
                    if(!viperYES)
                        vipertick=(viperEnc.getCurrentPosition()-vipval);
                    telemetry.addData("viper going down",true);
                    viper.setPower(0.82*n);
                    viperYES=true;
                }
                if((viperEnc.getCurrentPosition()-vipval)>=0) {
                    viper.setPower(0.0);
                }
                if(misumiYES){
                    if((misumiEnc.getCurrentPosition()-misval)<=-190  ){
                        misumi.setPower(-0.1);
                    }
                    else
                        telemetry.addData("going down",false);
                        misumi.setPower(0.1);
                }
                telemetry.addData("vipertick",vipertick);

                telemetry.update();
            }
//            viper.setPower(0.0);
        telemetry.addData("viper2",(viperEnc.getCurrentPosition()-vipval));
        telemetry.addData("vipertick",vipertick);
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

        while((viperEnc.getCurrentPosition()-vipval)>=-2425&&!viperYES){

            telemetry.addData("viper",(viperEnc.getCurrentPosition()-vipval));
            telemetry.addData("vipertick",vipertick);

            if((viperEnc.getCurrentPosition()-vipval)<=-1900){
                telemetry.addData("viperdecel",true);

                viper.setPower(-0.7*n);
            }
            else {
                viper.setPower(-2*n);
            }
            if((misumiEnc.getCurrentPosition()-misval)<=-188)
            {
                misumi.setPower(-0.1);
            }
            else
            {
                misumi.setPower(0.1);

            }
            telemetry.update();

//            telemetry.addData("viper", (viper.getCurrentPosition()-vipval));
//            telemetry.update();
        }
        telemetry.addData("vipertick",vipertick);

        telemetry.addData("viper3",(viperEnc.getCurrentPosition()-vipval));
        telemetry.update();
//        viper.setPower(0.0);
        if((misumiEnc.getCurrentPosition()-misval)<=-188)
        {
            misumi.setPower(-0.1);
        }
        else
        {
            misumi.setPower(0.1);

        }
//        misumi.setPower(0.1);
//        viper.setPower(0.
        while((viperEnc.getCurrentPosition()-vipval)<0)
        {
            viper.setPower(2*n);
            if((misumiEnc.getCurrentPosition()-misval)<=-188){
                misumi.setPower(-0.1);
            }
            else
            {
                misumi.setPower(0.1);

            }
        }
        telemetry.addData("viper3",(viperEnc.getCurrentPosition()-vipval));
        telemetry.update();
        viper.setPower(0.0);
//        misumi.setPower(0.12);
        while((misumiEnc.getCurrentPosition()-misval)>=-200){
            misumi.setPower(0.8);
        }
        misumi.setPower(0.0);
        telemetry.addData("misumi",misumiEnc.getCurrentPosition()-misval);
        telemetry.update();
        intakeExt.setPower(-0.45);
        sleep(30);
      intake.setPosition(0.3);
//      sleep(5000);
        //START OF FOR
        for(int i=0;i<5;i++){
            if(i==4)
                misumiTicks=20;
            else{
                misumiTicks+=72;
            }
            if(i==2){
                misumiTicks+=76;
            }
            else if(i==3){
                misumiTicks+=74;

            }
            else if(i==1){
                misumiTicks+=30;
            }
            intakeExt.setPower(-0.45);
            sleep(71);
            intake.setPosition(0.23);
//            sleep(10);
            intakeExt.setPower(0.0);
            sleep(270);
            misumi.setPower(2*n);
            while((misumiEnc.getCurrentPosition()-misval)>=-565){
//                telemetry.addData("misumi",misumiEnc.getCurrentPosition()-misval);
            }
            misumi.setPower(0.1);
            intakeTurn.setPosition(0.31405);
            intakeExt.setPower(1*n);
            while((intakeExtEnc.getCurrentPosition() - intval) <= -2) {

            }
            intakeExt.setPower(0.0);
            sleep(38);
            misumi.setPower(-0.2);
            while((misumiEnc.getCurrentPosition()-misval)<=-590){
//                telemetry.addData("misumi",misumiEnc.getCurrentPosition()-misval);
            }
            misumi.setPower(0.1);
            sleep(625);
            intake.setPosition(0.8);
            sleep(230);
            viper.setPower(-2*n);
            while((viperEnc.getCurrentPosition() - vipval) > -422){}
            intakeExt.setPower(-1 * n);
            if(i==4){

                intakeTurn.setPosition(0.5);
                sleep(30);
                misumi.setPower(0.03);
                intakeExt.setPower(0.0);
                viper.setPower(-2*n);
                while ((viperEnc.getCurrentPosition() - vipval) > -2440){
                    if((viperEnc.getCurrentPosition() - vipval) < -1940){viper.setPower(-0.6*n);
                    }

                }
//                viper.setPower(0.0);
//                sleep(10);
                viper.setPower(2*n);
                while((viperEnc.getCurrentPosition() - vipval) <0){
                    if((viperEnc.getCurrentPosition() - vipval)>-1700){
                        drive.setPoseEstimate(new Pose2d());
                        if(result=="0 Class 1"){

                        drive.followTrajectory(trajectoryback);
                    }
                        else if(result=="1 Class 2"){
                            drive.followTrajectory(trajectorystable);

                        }
                        else if(result=="2 Class 3"){
                            drive.followTrajectory(trajectoryfor);



                        }

                    }
                }
                viper.setPower(0.0);
                break;
            }

            misumi.setPower(0.8);
            sleep(55);
            misumi.setPower(0.1);
            intakeTurn.setPosition(0.965);
            intakeExt.setPower(0.0);
            sleep(410);
            intake.setPosition(0.8);
            intakeExt.setPower(-0.9* n);
            misumi.setPower(-0.3);
            while ((viperEnc.getCurrentPosition() - vipval) > -2455) {
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
                    if((viperEnc.getCurrentPosition()-vipval)<=-1940){

                        viper.setPower(-0.6*n);
                    }
                }

                intakeExt.setPower(-0.0);
                if ((misumiEnc.getCurrentPosition() - misval) >= misumiTicks) {
//                    telemetry.addData("stopped",true);
                    misumi.setPower(0.1);
//                    telemetry.update();
                }
            }
//            viper.setPower(0.1);


//            sleep(1000);

//            sleep(10);
            while ((viperEnc.getCurrentPosition() - vipval) < 0) {
                viper.setPower(2 * n);
                if((misumiEnc.getCurrentPosition() - misval) >= misumiTicks){
//                telemetry.addData("running",true);
//                telemetry.addData("running",(misumiEnc.getCurrentPosition() - misval));
//                telemetry.update();
                misumi.setPower(0.1);

                }

                telemetry.addData("viper",(viperEnc.getCurrentPosition() - vipval));
                telemetry.update();
            }
            viper.setPower(0.0);
//            telemetry.addData("time",System.currentTimeMillis()-start);
//            telemetry.addData("i",i);
            telemetry.update();
        }


        intakeTurn.setPosition(0.5);
        intake.setPosition(0.3);
}}