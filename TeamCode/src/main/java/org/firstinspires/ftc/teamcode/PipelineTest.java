package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.PropPipeline;
import org.firstinspires.ftc.teamcode.Globals.Location;
import org.firstinspires.ftc.vision.VisionPortal;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import com.acmerobotics.dashboard.config.Config;

@Config

@Autonomous(name = "PipelineTest")
public class PipelineTest extends LinearOpMode {

    private PropPipeline propPipeline;
    private VisionPortal portal;



    @Override
    public void runOpMode() throws InterruptedException {
        Globals.ALLIANCE = Location.BLUE;
        Globals.SIDE = Location.FAR;

        propPipeline = new PropPipeline();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(1920, 1080))
                .addProcessor(propPipeline)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        /*
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(camera, 0);

         */

        while (opModeInInit()) {

            telemetry.addData("Location", propPipeline.getLocation());
            telemetry.addData("leftZone", propPipeline.left.toString());
            telemetry.addData("centerZone", propPipeline.center.toString());
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Location", propPipeline.getLocation());
            telemetry.addData("leftZone", propPipeline.left.toString());
            telemetry.addData("centerZone", propPipeline.center.toString());
            telemetry.update();
        }
    }
}