package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

//@Disabled
@Autonomous(name="Vision Test")
public class Test2OpenCV extends LinearOpMode {
    private VisionPortal portal;
    private BluePropThreshold blue;

    @Override
    public void runOpMode() throws InterruptedException {

        blue = new BluePropThreshold();

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(1920, 1080))
                .addProcessor(blue)
                .build();


        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            telemetry.addData("Prop Position", blue.getPropPosition());
            telemetry.update();

            sleep(50);
        }


    }
}