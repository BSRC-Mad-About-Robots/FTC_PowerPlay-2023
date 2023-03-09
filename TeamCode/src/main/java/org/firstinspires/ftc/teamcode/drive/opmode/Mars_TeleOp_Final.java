package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
        intakeTurn = hardwareMap.get(Servo.class, "intakeTurn");
        CRServo turntable= hardwareMap.crservo.get("turntable");
        intake= hardwareMap.servo.get("intake");
        Encoder misumiEnc= new Encoder(hardwareMap.get(DcMotorEx.class, "misumi"));
        Encoder intakeExtEnc= new Encoder(hardwareMap.get(DcMotorEx.class, "intakeExt"));
        Encoder viperEnc= new Encoder(hardwareMap.get(DcMotorEx.class, "viper"));
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

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
        misumiEnc.setDirection(Encoder.Direction.REVERSE);
        //intakeTurn.setPosition(0.9);
        double misval=misumiEnc.getCurrentPosition();
        double intval=intakeExtEnc.getCurrentPosition();
         double vipval= viperEnc.getCurrentPosition();
//        intakeExtEnc.setMode(Encoder.RunMode.STOP_AND_RESET_ENCODER);
        intakeTurn.setPosition(0.5);
        waitForStart();

        while (!isStopRequested()) {
            telemetry.addData("Start",true);
            drive.setWeightedDrivePower(
                    new Pose2d(
                            gamepad1.right_stick_y*-1,
                            gamepad1.right_stick_x*-1,
                            ((gamepad1.right_trigger>0)?(-gamepad1.right_trigger):gamepad1.left_trigger)*1
                    )
            );

            drive.update();

            misumi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                misumi.setPower((gamepad2.left_trigger>0)? (gamepad2.left_trigger*0.45):(-gamepad2.right_trigger*-0.01));
            viper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            misumi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeExt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            adjust.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if(!(gamepad2.right_trigger>0) && !(gamepad2.left_trigger>0))
            {
                misumi.setPower(0.1);
            }
            if(gamepad2.b) {

                intake.setPosition(0.85);
                if((misumiEnc.getCurrentPosition()-misval)>50&& ((intakeExtEnc.getCurrentPosition()-intval)>=-200)){
                    misumi.setPower(0.6);
                    sleep(100);
                    intake.setPosition(0.5);
                }

//                sleep(300);
                if(b)
                {
                    ticksinext=-270;
                    telemetry.addData("ticks",ticksinext);
                }
                while((intakeExtEnc.getCurrentPosition()-intval)>=ticksinext)
                {

                        intakeExt.setPower(-0.5);


                    telemetry.addData("ticksdd",ticksinext);

                    if((misumiEnc.getCurrentPosition()-misval)>100)
                    {
                        misumi.setPower(0.1);
                        sleep(150);
                        intakeTurn.setPosition(0.976);
                        while((misumiEnc.getCurrentPosition()-misval)>0){
                            misumi.setPower(-0.2);
                    }
//                    telemetry.addData("io",intakeTurn.getPosition());
//                    telemetry.update();
                }

                }
                intakeExt.setPower(0.0);
                if((intakeExtEnc.getCurrentPosition()-intval)>=-200)
                { misumi.setPower(-0.2);}
//
                if((intakeExtEnc.getCurrentPosition()-intval)<=-500&& (intakeTurn.getPosition()<0.5))
                {
                    misumi.setPower(0.6);

                    sleep(200);
                    intakeTurn.setPosition(0.976);
                    sleep(200);
                    misumi.setPower(-0.2);

                    while((misumiEnc.getCurrentPosition()-misval)>-40){

                    }
                    misumi.setPower(0.1);
                }
                intakeTurn.setPosition(0.976);
                b=true;
                intake.setPosition(0.8);
            }
//            }

            else if(gamepad2.x){
                intake.setPosition(0.2);
                sleep(880);
                sleep(30);
                misumi.setPower(0.6);
                sleep(55);
                intakeTurn.setPosition(0.31);
                intakeExt.setPower(0.4);
                while((intakeExtEnc.getCurrentPosition()-intval)<=0)
                {
//                    sleep(70);
                    telemetry.addData("intakeExt",(intakeExtEnc.getCurrentPosition()-intval));
//                    telemetry.addData("boolean",b);
                    telemetry.update();
                   if((misumiEnc.getCurrentPosition()-misval)>=525)
                   {telemetry.addData("intakeExt",(intakeExtEnc.getCurrentPosition()-intval));
//                    telemetry.addData("boolean",b);
                       telemetry.update();
                       misumi.setPower(0.12);
                   }
                    telemetry.addData("intakeExt",(intakeExtEnc.getCurrentPosition()-intval));
//                    telemetry.addData("boolean",b);
                    telemetry.update();

                    intakeExt.setPower(0.4);

                }

                intakeExt.setPower(0.0);
                sleep(600);
                intake.setPosition(0.8);
                intakeExt.setPower(-0.6);
                sleep(20);
                intakeExt.setPower(0.0);
            }
            if(gamepad2.left_stick_button){
                viper.setPower(0.7);
                while((viper.getCurrentPosition()-vipval)<0){}viper.setPower(0.0);
            }
            if(gamepad2.a)
            {
                intake.setPosition(0.3);
            }
            else if(gamepad2.y)
            {
                intake.setPosition(0.85);
            }
            intakeExt.setPower(gamepad2.right_stick_y*0.8);
            viper.setPower(gamepad2.left_stick_y);
//            if(gamepad2.right_stick_x==0){
//                viper.setPower(0.15);
//            }
            misumi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if(gamepad2.left_bumper)
            {
                turntable.setPower(0.9);
            }
            else if(gamepad2.right_bumper)
            {
                turntable.setPower(-0.9);
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
            telemetry.addData("misumi",misumiEnc.getCurrentPosition()-misval);
            telemetry.addData("intakeExt",intakeExtEnc.getCurrentPosition()-intval);
            telemetry.addData("intakeTurn",intakeTurn.getPosition());
            telemetry.addData("boolean",b);
            telemetry.addData("viper",viperEnc.getCurrentPosition()-vipval);
            telemetry.update();
        }
    }
}
