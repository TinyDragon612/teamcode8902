package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

//road runner imports

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.oldCode.TeleOpMain23;

import java.util.List;

//camera imports

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Disabled
@Autonomous(name = "AutonAprilTag", group = "")
public class AutonAprilTag extends TeleOpMain23 {

    //camera and image detection stuff
    private WebcamName camera;
    private VisionPortal visionPortal;
    private VisionProcessor visionProcessors;

    private TfodProcessor tfodProcessor;

    private AprilTagProcessor aprilTagProcessor;

    @Override
    public void runOpMode(){

        camera = hardwareMap.get(WebcamName.class, "Webcam1");
        visionPortal = VisionPortal.easyCreateWithDefaults(camera, visionProcessors);

        tfodProcessor = TfodProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(camera, tfodProcessor);
        List<Recognition> recognitions = tfodProcessor.getRecognitions();

        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(camera, aprilTagProcessor);
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

        waitForStart();

        if (opModeIsActive()){
            for (Recognition recognition : recognitions){
                String label = recognition.getLabel();

                float confidence = recognition.getConfidence();

                telemetry.addLine("Recognized Object:" + label);
                telemetry.addLine("Confidence:" + confidence);
            }

            for(AprilTagDetection detection : detections){
                int id = detection.id;

                AprilTagPoseFtc tagPose = detection.ftcPose;

                telemetry.addLine("Detected atg ID: " + id);
                telemetry.addLine("Distance to tag: " + tagPose.range);
                telemetry.addLine("Bearing to tag: " + tagPose.bearing);
                telemetry.addLine(" Angle of tag:" + tagPose.yaw);

            }
        }



    }







}