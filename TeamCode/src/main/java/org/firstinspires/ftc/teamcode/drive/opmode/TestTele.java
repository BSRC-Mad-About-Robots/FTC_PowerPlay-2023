package org.firstinspires.ftc.teamcode.drive.opmode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp(name="TestTele",group="Final")
public class TestTele extends OpMode
{
    // CREATING HARDWARE COMPONENT OBJECTS
    //WheelBase
    private DcMotor FL;
    private DcMotor BL;
    private DcMotor FR;
    private DcMotor BR;
    private DcMotor EL, ER, EM;
//    BNO055IMU imu;
//    NormalizedColorSensor colorSensor;
//    DcMotor Bask;


    // CREATING TIME COUNTER
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init()
    {
        // SHOWING THAT THE HARDWARE IS GETTING INITIALIZED ON DRIVER STATION VIA        TELEMETRY
        telemetry.addData("Status", "INITIALIZING");
        telemetry.update();

        //Initialize in hardware map to use in configurations
        // DECLARING CONFIGURATION NAMES OF HARDWARE COMPONENTS
        FL = hardwareMap.dcMotor.get("FL");
        BL = hardwareMap.dcMotor.get("BL");
        FR = hardwareMap.dcMotor.get("FR");
        BR = hardwareMap.dcMotor.get("BR");
//        ER = hardwareMap.dcMotor.get("FR");
//        EL = hardwareMap.dcMotor.get("BL");
//        EM = hardwareMap.dcMotor.get("FL");

//        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        //Give initial direction because of bevel gear
//        FR.setDirection(DcMotorSimple.Direction.REVERSE);
//        BR.setDirection(DcMotorSimple.Direction.REVERSE);
//        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //EL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //ER.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //EM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    @SuppressLint("SuspiciousIndentation")
    @Override
    public void loop()
    {


        // telemetry.addData("EL=",EL.getCurrentPosition());
        // telemetry.addData("ER=",ER.getCurrentPosition());
        // telemetry.addData("EM=",EM.getCurrentPosition());
        // telemetry.update();



        //double leftstickx = 0;
        //double leftsticky = 0;
        //double dpadpower = 0.25;

        float vertical;
        float horizontal;
        float pivot;
        float gain = 2;
        double power_factor = -0.4;
//        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
//        if (colorSensor instanceof  SwitchableLight)
//            ((SwitchableLight)colorSensor).enableLight(true);
//        colorSensor.setGain(gain);
        vertical = -gamepad1.right_stick_y;
        horizontal = gamepad1.right_stick_x;

        if(gamepad1.left_trigger>0){
            pivot = -gamepad1.left_trigger;
        }else if(gamepad1.right_trigger>0){
            pivot = +gamepad1.right_trigger;
        }else{
            pivot = 0;
        }
        if(gamepad1.x)
        FR.setPower((-pivot + (vertical - horizontal)) * power_factor);
       if(gamepad1.a)

        BR.setPower((-pivot + vertical + horizontal) * power_factor);
        if(gamepad1.b)
        FL.setPower((pivot + vertical + horizontal) * power_factor);
        if (gamepad1.y)
        BL.setPower((pivot + (vertical - horizontal)) * power_factor);

        // Put loop blocks here.
        telemetry.addData("Motor", "power FL %f, FR %f, BL %f, BR %f", FL.getPower(), FR.getPower(), BL.getPower(), BR.getPower());
        telemetry.addData("Motor", "vertical %f, horizontal %f, pivot %f", vertical, horizontal, pivot);
        telemetry.addData("Motor", "gamepad1.left_stick %f %f", gamepad1.left_stick_x, gamepad1.left_stick_y);
        telemetry.addData("Motor", "gamepad1.right_stick %f %f", gamepad1.right_stick_x, gamepad1.right_stick_y);
        telemetry.update();

          /*if(gamepad1.dpad_up)
            {
                leftsticky = dpadpower;
            }
            else if(gamepad1.dpad_right)
            {
                leftstickx = dpadpower;
            }
            else if(gamepad1.dpad_down)
            {
                leftsticky = -dpadpower;
            }
             else if(gamepad1.dpad_left)
            {
                leftstickx = -dpadpower;
            }
            else
            {
              leftstickx = gamepad1.left_stick_x;
              leftsticky = -gamepad1.left_stick_y;


            }*/



    }
}
//   public void sleepTeleOp(double runTime, double reqTime) {
//    while (runtime.seconds() < (reqTime + runTime)) {}




