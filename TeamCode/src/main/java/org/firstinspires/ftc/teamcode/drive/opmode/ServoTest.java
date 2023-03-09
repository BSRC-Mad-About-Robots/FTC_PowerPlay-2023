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
@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "drive")
public class ServoTest extends LinearOpMode {
    public Servo intake=null;
    public void runOpMode() throws InterruptedException {
    intake= hardwareMap.servo.get("intake");
    while (!isStopRequested()) {
        if(gamepad2.a)
        {
            telemetry.addData("a",true);
            telemetry.update();
            intake.setPosition(0.4);
        }
        else if(gamepad2.y)
        {
            telemetry.addData("Y",true);
            telemetry.update();
            intake.setPosition(0.85);
        }
    }
}}
