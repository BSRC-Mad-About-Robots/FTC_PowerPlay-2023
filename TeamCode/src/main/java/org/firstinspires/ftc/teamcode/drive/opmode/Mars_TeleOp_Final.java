package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.teamcode.util.Encoder.Direction.FORWARD;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "drive")
public class Mars_TeleOp_Final extends LinearOpMode {
    BNO055IMU imu;
    public Servo intakeTurn=null;
    public Servo intake=null;
    public boolean b=false;
    public int ticksinext=-685;
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "FL");
        DcMotorEx leftRear = hardwareMap.get(DcMotorEx.class, "BL");
        DcMotorEx rightRear = hardwareMap.get(DcMotorEx.class, "BR");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "FR");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Encoder leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "BR"));
//        Encoder rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "BL"));
        Encoder frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "FR"));
        DcMotor misumi=hardwareMap.dcMotor.get("misumi");
        DcMotor intakeExt=hardwareMap.dcMotor.get("intakeExt");
        DcMotor viper=hardwareMap.dcMotor.get("viper");
        DcMotor adjust=hardwareMap.dcMotor.get("adjust");
//        Servo intake=hardwareMap.get(Servo.class, "intake");
        Encoder turntableEnc = new Encoder(hardwareMap.get(DcMotorEx.class, "FR"));
        int turnval = turntableEnc.getCurrentPosition();
        boolean x=false;
        intakeTurn = hardwareMap.get(Servo.class, "intakeTurn");
        CRServo turntable= hardwareMap.crservo.get("turntable");
        intake= hardwareMap.servo.get("intake");
//        Encoder turntableEnc= new Encoder(hardwareMap.get(DcMotorEx.class, "FR"));
//        Encoder misumiEnc= new Encoder(hardwareMap.get(DcMotorEx.class, "misumi"));
//        Encoder intakeExtEnc= new Encoder(hardwareMap.get(DcMotorEx.class, "intakeExt"));
//        Encoder viperEnc= new Encoder(hardwareMap.get(DcMotorEx.class, "viper"));
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        NormalizedColorSensor colorSensor;
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
//        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        parameters.loggingEnabled      = false;
        parameters.loggingTag          = "IMU";
        // parameters.accelerationIntegrationAlgorit[m = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
//        Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
        viper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        misumi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeExt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        adjust.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        misumi.setDirection(DcMotorSimple.Direction.REVERSE);
//        misumiEnc.setDirection(Encoder.Direction.REVERSE);
//        //intakeTurn.setPosition(0.9);
//        double misval=misumiEnc.getCurrentPosition();
//        double intval=intakeExtEnc.getCurrentPosition();
//         double vipval= viperEnc.getCurrentPosition();
//        double turnval= turntableEnc.getCurrentPosition();
//        intakeExtEnc.setMode(Encoder.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
//        intakeTurn.setPosition(0.55);
        while (!isStopRequested()) {
            telemetry.addData("Start",true);

            drive.setWeightedDrivePower(

                    new Pose2d(
                            gamepad1.right_stick_y*-0.8,
                            gamepad1.right_stick_x*-0.5,
                            ((gamepad1.right_trigger>0)?(-gamepad1.right_trigger):gamepad1.left_trigger)*0.7
                    )
            );

            drive.update();
//            if (colorSensor instanceof DistanceSensor && ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM)<2.0)
//            {
//                telemetry.addData("ir",true);
//                telemetry.update();
//                adjust.setPower(-1.0);
////                CarR.setPower(-1.0);
//            }
//            else{
//                adjust.setPower(0.0);
//            }
            misumi.setPower((gamepad2.left_trigger>0.15)? (gamepad2.left_trigger*0.8):(gamepad2.right_trigger*-0.1));
            if(!(gamepad2.right_trigger>0) && !(gamepad2.left_trigger>0))
            {
                misumi.setPower(0.1);
            }
//            if(gamepad2.b&&gamepad1.right_stick_x==0) {
//
//                intake.setPosition(0.85);
//                adjust.setPower(0.0);
//                if((misumiEnc.getCurrentPosition()-misval)>50&& ((intakeExtEnc.getCurrentPosition()-intval)>=-200)){
//                    misumi.setPower(0.6);
//                    sleep(100);
//                    intake.setPosition(0.5);
//                }
//
////                sleep(300);
//                if(b)
//                {
//                    ticksinext=-270;
//                    telemetry.addData("ticks",ticksinext);
//                }
//                while((intakeExtEnc.getCurrentPosition()-intval)>=ticksinext)
//                {
//
//                        intakeExt.setPower(-0.5);
//
//
//                    telemetry.addData("ticksdd",ticksinext);
//
//                    if((misumiEnc.getCurrentPosition()-misval)>100)
//                    {
//                        misumi.setPower(0.1);
//                        sleep(150);
//                        intakeTurn.setPosition(0.976);
//                        while((misumiEnc.getCurrentPosition()-misval)>0){
//                            misumi.setPower(-0.2);
//                    }
////                    telemetry.addData("io",intakeTurn.getPosition());
////                    telemetry.update();
//                }
//
//                }
//                intakeExt.setPower(0.0);
//                if((intakeExtEnc.getCurrentPosition()-intval)>=-200)
//                { misumi.setPower(-0.2);}
////
//                if((intakeExtEnc.getCurrentPosition()-intval)<=-500&& (intakeTurn.getPosition()<-0.3))
//                {
//                    misumi.setPower(0.6);
//
//                    sleep(200);
//                    intakeTurn.setPosition(0.976);
//                    sleep(200);
//                    misumi.setPower(-0.2);
//
//                    while((misumiEnc.getCurrentPosition()-misval)>-40){
//
//                    }
//                    misumi.setPower(0.1);
//                }
//                intakeTurn.setPosition(0.976);
//                b=true;
//                intake.setPosition(0.8);
//                adjust.setPower(0.0);
//            }
////            }
//
//            else if(gamepad2.x&&leftFront.getPower()==0){
//                x=true;
//                misumiEnc.setDirection(Encoder.Direction.FORWARD);
//                intake.setPosition(0.2);
//                adjust.setPower(-1.0);
//                sleep(880);
//                sleep(30);
//                misumi.setPower(0.9);
//                sleep(55);
//                intakeTurn.setPosition(0.31405);
//                sleep(700);
//                intakeExt.setPower(0.4);
//                while((intakeExtEnc.getCurrentPosition()-intval)<=-2) {
////                    sleep(70);
//                    telemetry.addData("intakeExt", (intakeExtEnc.getCurrentPosition() - intval));
////                    telemetry.addData("boolean",b);
//                    telemetry.update();
//                    if ((misumiEnc.getCurrentPosition() - misval) >= 585) {
//                        telemetry.addData("intakeExt", (intakeExtEnc.getCurrentPosition() - intval));
////                    telemetry.addData("boolean",b);
//                        telemetry.update();
//                        misumi.setPower(0.12);
//                    }
//                    telemetry.addData("intakeExt", (intakeExtEnc.getCurrentPosition() - intval));
////                    telemetry.addData("boolean",b);
//                    telemetry.update();
//                }
//                    intakeExt.setPower(0.4);
//                    intake.setPosition(0.85);
//                adjust.setPower(-0.0);
//                    sleep(600);
//                    intakeExt.setPower(-0.7);
//                    sleep(250);
//                    misumi.setPower(0.8);
//                    sleep(20);
//                    misumi.setPower(0.1);
//                    intakeTurn.setPosition(0.965);
//                    intakeExt.setPower(0.0);
//                    sleep(410);
//                    intake.setPosition(1);
//                    adjust.setPower(0.0);
//                    intakeExt.setPower(-0.7);
//                    misumi.setPower(-0.4);
//                    while ((intakeExtEnc.getCurrentPosition() - intval) >= -350) {
//                        if((intakeExtEnc.getCurrentPosition() - intval) <= -200){
//                            intakeExt.setPower(-0.4);
//                        }
//                        if ((misumiEnc.getCurrentPosition() - misval) <= 20) {
////                        telemetry.addData("stopped",true);
//                            misumi.setPower(-0.4);
////                        telemetry.update();
//                        }
//                        else{
//                            misumi.setPower(0.1);
//                        }
//
//                    }
//                    while((misumiEnc.getCurrentPosition()-misval)<=20)
//                    {
//                        misumi.setPower(-0.4);
//                    }
//                misumi.setPower(0.1);
//                intakeExt.setPower(0.0);
//                sleep(600);
//                intakeExt.setPower(-0.6);
//                sleep(10);
//                intakeExt.setPower(0.0);
//                while((misumiEnc.getCurrentPosition()-misval)<=0)
//                {
//                    misumi.setPower(-0.4);
//                }
//                misumi.setPower(0.1);
//                misumiEnc.setDirection(Encoder.Direction.REVERSE);
//                x=false;
//                telemetry.addData("x",x);
//                telemetry.update();
//            }
            if(gamepad2.dpad_up){
                intakeTurn.setPosition(0.325);
            }
            else if(gamepad2.dpad_down){
                intakeTurn.setPosition(0.965);
            }
//            if(gamepad2.left_stick_button){
//                viper.setPower(1);
//                while((viper.getCurrentPosition()-vipval)<0){}viper.setPower(0.0);
//            }
//            if(gamepad2.right_stick_button){
//                intake.setPosition(0.23);
//                sleep(300);
//                misumi.setPower(0.8);
//                intakeExt.setPower(0.6);
//                while((intakeExtEnc.getCurrentPosition())<=0){
//                while((misumiEnc.getCurrentPosition())<=555){
//                    intakeTurn.setPosition(0.55);
//                }}
//                misumi.setPower(0.1);
//                intakeExt.setPower(0.0);
//                while((misumiEnc.getCurrentPosition())<=555) {
//                    intakeTurn.setPosition(0.55);
//                    misumi.setPower(0.8);
//                }
//                intakeTurn.setPosition(0.55);
//                misumi.setPower(0.0);
//            }
            if(gamepad2.a)
            {
                intake.setPosition(0.3);
                adjust.setPower(-1.0);
            }
            else if(gamepad2.y)
            {
                intake.setPosition(0.85);
                adjust.setPower(0.0);
            }
            intakeExt.setPower(gamepad2.right_stick_y*0.8);
            viper.setPower(gamepad2.left_stick_y*0.7);
//            if(gamepad2.right_stick_x==0){
//                viper.setPower(0.15);
//            }
            misumi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if(gamepad2.left_bumper)
            {
                turntable.setPower(0.9);
//                telemetry.addData("turn",turntableEnc.getCurrentPosition()-turnval);
                telemetry.update();
            }
            else if(gamepad2.right_bumper)
            {
                turntable.setPower(-0.9);
//                telemetry.addData("turn",turntableEnc.getCurrentPosition()-turnval);
                telemetry.update();
            }
            else
                turntable.setPower(0.0);
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("left", leftEncoder.getCurrentPosition());
//            telemetry.addData("right", rightEncoder.getCurrentPosition());
            telemetry.addData("back", frontEncoder.getCurrentPosition());
            telemetry.addData("imu",drive.getPoseVelocity());
//            telemetry.addData("misumi",misumiEnc.getCurrentPosition()-misval);
//            telemetry.addData("intakeExt",intakeExtEnc.getCurrentPosition()-intval);
//            telemetry.addData("intakeTurn",intakeTurn.getPosition());
//            telemetry.addData("boolean",b);
//            telemetry.addData("viper",viperEnc.getCurrentPosition()-vipval);
//            telemetry.addData("turnval",turntableEnc.getCurrentPosition()-turnval);

            telemetry.update();
        }
    }
}
