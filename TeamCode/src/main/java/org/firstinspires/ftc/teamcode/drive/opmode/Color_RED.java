package org.firstinspires.ftc.teamcode.drive.opmode;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp(group = "drive")
public class Color_RED extends LinearOpMode {


        BNO055IMU imu;
        public Servo intakeTurn=null;
        public Servo intake=null;
        public boolean b=false;
        public int ticksinext=-685;
//    DcMotor adjust=hardwareMap.dcMotor.get("adjust");
final float[] hsvValues = new float[3];
    @Override
        public void runOpMode() throws InterruptedException {
            NormalizedColorSensor colorSensor;
            colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
            waitForStart();

            while(opModeIsActive()){
                NormalizedRGBA colors = colorSensor.getNormalizedColors();
                Color.colorToHSV(colors.toColor(), hsvValues);
            if(hsvValues[0]<100&&hsvValues[0]>10){
                telemetry.addData("dectected red",true);
//                telemetry.addData("red",colors.red);
                telemetry.update();
//                adjust.setPower(-1.0);
            }
            else if(hsvValues[0]>100){
                telemetry.addData("detected blue",false);
//                telemetry.addData("red",colors.red);
                telemetry.update();
            }
            sleep(100);}
        }
}
