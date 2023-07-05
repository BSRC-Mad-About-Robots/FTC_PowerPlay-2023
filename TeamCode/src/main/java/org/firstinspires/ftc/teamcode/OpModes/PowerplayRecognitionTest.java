package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//import org.firstinspires.ftc.teamcode.autonomous.AutoDot;
//import org.firstinspires.ftc.teamcode.autonomous.AutoRoute;
//import org.firstinspires.ftc.teamcode.bots.OutreachBot;
import org.firstinspires.ftc.teamcode.skills.PowerplayDetector;
//import org.firstinspires.ftc.teamcode.skills.RingDetector;

// Control Hub ADB Terminal Command for Reference
// adb.exe connect 192.168.43.1:5555




@TeleOp(name = "Powerplay SleeveTest", group = "Robot15173")
//@Disabled
public class PowerplayRecognitionTest extends LinearOpMode
{
    // Declare OpMode members.
    private PowerplayDetector rf = null;
    private String result = "";
    //private OutreachBot bot = new OutreachBot();

    @Override
    public void runOpMode() {
        try {
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
                    //telemetry.addData("DetectionTest result", result);
                    //telemetry.update();
                    sleep( 1000);
                }
            } catch (Exception ex) {
                telemetry.addData("Error", String.format("Unable to initialize Detector. %s", ex.getMessage()));
                telemetry.update();
                sleep(5000);
                return;
            }

            // Wait for the game to start (driver presses PLAY)
            //telemetry.update();

            waitForStart();

            result = rf.getResult();

            telemetry.addData("FINAL Detection result", result);
            telemetry.update();

            rf.stopDetection();
//            rf = null;
            //sleep( 2000);


            // run until the end of the match (driver presses STOP)
//            while (opModeIsActive()) {
//                //move the bot
//                double drive = gamepad1.left_stick_y;
//                //bot.move(drive);
//
//                //show recognition result
//                //telemetry.addData("FINAL FINAL Detection result", result);
//                //telemetry.update();
//            }
        } catch (Exception ex) {
            telemetry.addData("Init Error", ex.getMessage());
            telemetry.update();
        } finally {
            if (rf != null) {
                rf.stopDetection();
            }
        }
    }
}
