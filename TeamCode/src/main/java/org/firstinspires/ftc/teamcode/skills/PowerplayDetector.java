 package org.firstinspires.ftc.teamcode.skills;

 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.eventloop.opmode.OpMode;
 import com.qualcomm.robotcore.hardware.HardwareMap;
 import com.qualcomm.robotcore.util.ElapsedTime;

 import org.firstinspires.ftc.robotcore.external.Telemetry;
 //import org.firstinspires.ftc.teamcode.autonomous.AutoDot;
 //import org.firstinspires.ftc.teamcode.autonomous.AutoRoute;
 import org.firstinspires.ftc.teamcode.tfrec.Detector;
 import org.firstinspires.ftc.teamcode.tfrec.classification.Classifier;

 import java.util.HashMap;
 import java.util.List;
 import java.util.Map;
 import java.util.Map.Entry;

 public class PowerplayDetector implements Runnable{
     Telemetry telemetry;
     private Detector tfDetector = null;
     private HardwareMap hardwareMap;

     private boolean isRunning = true;


     private String modelFileName = "LATEST FINAL SLEEVE RED MODEL.tflite";//"croppedRingRec.tflite";
     private String labelFileName = "SLEEVE labels.txt";//"croppedLabels.txt";
     private static Classifier.Model MODEl_TYPE = Classifier.Model.FLOAT_EFFICIENTNET;
     private static final String LABEL_A = "0 Class 1";
     private static final String LABEL_B = "1 Class 2";
     private static final String LABEL_C = "2 Class 3";

     private String result = LABEL_B; //just a default value.

     private LinearOpMode caller = null;



     public PowerplayDetector(HardwareMap hMap, LinearOpMode caller, Telemetry t) throws Exception {
         hardwareMap = hMap;
         telemetry = t;
         initDetector();
         activateDetector();
         this.caller = caller;

     }

     public PowerplayDetector(HardwareMap hMap, LinearOpMode caller, Telemetry t, String model, String labels) throws Exception {
         hardwareMap = hMap;
         telemetry = t;
         setModelFileName(model);
         setLabelFileName(labels);
         initDetector();
         activateDetector();
         this.caller = caller;
     }

    public String FindBestOfN(String[] results)
    {
        String result = new String();
        Map<String, Integer> hash = new HashMap<String, Integer>();
        for (String res : results) {
            if (hash.containsKey(res))
                hash.put(res, hash.get(res) + 1);
            else
                hash.put(res, 1);
        }

        int max = 0;
        for (Entry<String, Integer> item : hash.entrySet())
        {
            if (max < item.getValue())
            {
                result = item.getKey();
                max = item.getValue();
            }
        }

        return result;
    }


     public void detectSleeveThread()
     {
         int bestOfN = 5;
         try {
         ElapsedTime runtime = new ElapsedTime();
         runtime.reset();

         while (isRunning) {
             if (tfDetector != null) {
                 String[] result = new String[bestOfN];
                 double[] confidence = new double[bestOfN];
                 for (int i = 0; i < bestOfN; i++)
                 {
                     List<Classifier.Recognition> results = tfDetector.getLastResults();
                     if (results == null || results.size() == 0) {
                         telemetry.addData("Nada", "No results");
                         telemetry.update();
                         //Thread.sleep(1000);
                     } else {
                         //telemetry.addData("GOT RESULTS", results.size());
                         //telemetry.update();
                         //Thread.sleep(1000);

                         double max_confidence = 0.0;
                         String max_label = "";
                         for (Classifier.Recognition r : results) {
                             if (r.getConfidence() > max_confidence) {
                                 max_confidence = r.getConfidence();
                                 max_label = r.getTitle();
                             }
                         }

                         if (max_confidence >= 0.3) {
                             //telemetry.addData("Confidence", max_confidence);
                             //telemetry.addData("Label", max_label);

                             if (max_label.contains(LABEL_A)) {
                                 result[i] = LABEL_A;
                             } else if (max_label.contains(LABEL_B)) {
                                 result[i] = LABEL_B;
                             } else if (max_label.contains(LABEL_C)) {
                                 result[i] = LABEL_C;
                             }
                             //telemetry.update();
                             Thread.sleep(100);
                         }
                         confidence[i] = max_confidence;
                     }
                 }

                 for (int jj = 0; jj < bestOfN; jj++)
                 {
                     telemetry.addData("BEST5", result[jj]);
                     telemetry.addData("BEST5", confidence[jj]);
                 }

                 this.result = FindBestOfN(result);

                 double mean_confidence = 0.0;
                 for (int jj = 0; jj < bestOfN; jj++)
                 {
                     mean_confidence += confidence[jj];
                 }
                 mean_confidence = mean_confidence / bestOfN;

                 String final_result = LABEL_B;
                 if (mean_confidence > 0.8)
                 {
                     if ((this.result == LABEL_A) || this.result == LABEL_C)
                        final_result = this.result;
                 }

                 this.result = final_result;
                 telemetry.addData("RESULT", this.result);
                 telemetry.update();
                 Thread.sleep(100);
             }
             else {
                 telemetry.addData("detectRingThread", "tfDetector is null");
                 telemetry.update();
             }
             telemetry.update();
         }

         }
         catch(InterruptedException e)
         {}
     }


     public void initDetector() throws Exception {
         tfDetector = new Detector(MODEl_TYPE, getModelFileName(), getLabelFileName(), hardwareMap.appContext, telemetry);
     }

     protected void activateDetector() throws Exception {
         if (tfDetector != null) {
             tfDetector.activate();
         }
         telemetry.addData("Info", "TF Activated");
     }


     public void stopDetection() {
         stopThread();
         if (tfDetector != null) {
             tfDetector.stopProcessing();
         }
         tfDetector = null;
     }

     public void stopThread() {
         isRunning = false;
     }

     @Override
     public void run() {
         while(isRunning) {
             detectSleeveThread();
         }
     }


     public String getModelFileName() {
         return modelFileName;
     }

     public void setModelFileName(String modelFileName) {
         this.modelFileName = modelFileName;
     }

     public String getLabelFileName() {
         return labelFileName;
     }

     public void setLabelFileName(String labelFileName) {
         this.labelFileName = labelFileName;
     }

     public String getResult() {
         return result;
     }

     public void setResult(String result) {
         this.result = result;
     }
 }
