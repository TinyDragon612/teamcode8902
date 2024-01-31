package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.vision.ABetterThreshold;
import org.firstinspires.ftc.teamcode.vision.RedPropThreshold;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@Disabled
@Config
@Autonomous(group = "drive")
public class ThresholdRace extends LinearOpMode {

    private VisionPortal portal;
    private ABetterThreshold better;

    private RedPropThreshold ruby;

    ElapsedTime BetterTimer = new ElapsedTime();
    ElapsedTime RubyTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        better = new ABetterThreshold();
        ruby = new RedPropThreshold();

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(1920, 1080))
                .addProcessor(ruby)
                .addProcessor(better)
                .build();

        double betterDone = 0;
        double rubyDone = 0;

        waitForStart();
        BetterTimer.reset();
        RubyTimer.reset();

        if (opModeIsActive() && !isStopRequested()) {
            while (opModeIsActive()) {

                    telemetry.addData("Prop Scan Better: ", better.getPropPosition());
                    telemetry.addData("Prop Time Better: ", BetterTimer.seconds());
                    telemetry.addData(" ", " ");
                    telemetry.addData("Prop Scan Ruby: ", ruby.getPropPosition());
                    telemetry.addData("Prop Time Ruby: ", RubyTimer.seconds());
                    telemetry.addData(" ", " ");
                    telemetry.addData("Scan Done Ruby: ", betterDone);
                    telemetry.addData("Scan Done Ruby: ", rubyDone);
                    telemetry.update();


                    if(better.getPropPosition() == "right"){
                        betterDone = BetterTimer.seconds();
                    }

                    if(better.getPropPosition() == "right"){
                        rubyDone = RubyTimer.seconds();
                    }


                    if(better.getPropPosition() == "right" && ruby.getPropPosition() == "right"){
                        portal.stopStreaming();
                    }


            }
        }

    }
}
