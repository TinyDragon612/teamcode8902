package org.firstinspires.ftc.teamcode.test;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import org.firstinspires.ftc.teamcode.vision.PropPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

//MODIFIED FROM BLUE LEFT
@Config
@Autonomous(name= "wacky stacky", group = "drive")
public class stackTest extends LinearOpMode {

    private DcMotorEx slide1, slide2; //drawers

    private TouchSensor magnetic, magnetic2;

    private ServoImplEx swoosh1, swoosh2, flop1, flop2, pinch1, pinch2, drop, launcher;

    private DcMotorEx spin;


    private int errorBound = 60;

    int height;

    ElapsedTime CVTimer = new ElapsedTime();
    public ElapsedTime drawerTimer = new ElapsedTime();
    ElapsedTime servoTimer = new ElapsedTime();
    ElapsedTime spinTimer = new ElapsedTime();

    public static double FLIP_TIME = 1.5;

    int count;

    public enum State{
        START,
        TRAJECTORY,
        PIXEL,
        TRAJECTORY2,
        DOWN,
        SETTLE,
        DRAWER_START,
        DRAWER_FLIP_OUT,
        DRAWER_FLIP_IN,
        DRAWER_RETRACT,
        DRAWER_SETTLE,
        RELEASE,
        TRAJECTORY3,
        LOCK_IN,
        TRAJECTORY4
    }

    public enum StrikePosition{
        MIDDLE,
        LEFT,
        RIGHT
    }

    public StrikePosition strikePos = StrikePosition.MIDDLE;

    public State currentState = State.START;

    public static double CV_RUNTIME = 8;
    private VisionPortal portal;
    private PropPipeline propPipeline = new PropPipeline();;

    private DcMotorEx.ZeroPowerBehavior brake = DcMotorEx.ZeroPowerBehavior.BRAKE;

    @Override
    public void runOpMode() throws InterruptedException {

        slide1 = hardwareMap.get(DcMotorEx.class, "slide1");
        slide2 = hardwareMap.get(DcMotorEx.class, "slide2");
        magnetic = hardwareMap.get(TouchSensor.class, "magnetic");
        magnetic2 = hardwareMap.get(TouchSensor.class, "magnetic2");
        swoosh1 = hardwareMap.get(ServoImplEx.class, "swoosh1");
        swoosh2 = hardwareMap.get(ServoImplEx.class, "swoosh2");
        flop1 = hardwareMap.get(ServoImplEx.class, "flop1");
        flop2 = hardwareMap.get(ServoImplEx.class, "flop2");
        pinch1 = hardwareMap.get(ServoImplEx.class, "pinch1");
        pinch2 = hardwareMap.get(ServoImplEx.class, "pinch2");
        drop = hardwareMap.get(ServoImplEx.class, "drop");
        launcher = hardwareMap.get(ServoImplEx.class, "launcher");
        spin = hardwareMap.get(DcMotorEx.class, "spin");

        slide1.setDirection(DcMotorEx.Direction.FORWARD);
        slide2.setDirection(DcMotorEx.Direction.REVERSE);

        slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slide1.setTargetPosition(0);
        slide2.setTargetPosition(0);

        slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        flop1.setPosition(0.97);
        flop2.setPosition(0.03);

        swoosh1.setPosition(.1325);
        swoosh2.setPosition(0.0675);

        pinch1.setPosition(0);
        pinch2.setPosition(1);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);

        TrajectorySequence basic = drive.trajectorySequenceBuilder(startPose)
                .forward(26)
                .addDisplacementMarker(() -> {
                    spinTimer.reset();
                    while(spinTimer.seconds() < 1.5){
                        spin.setPower(-0.9);
                    }
                    if(spinTimer.seconds() > 1.5){
                        spin.setPower(0);
                    }
                })
                .back(7.5)
                .turn(Math.toRadians(-93))
                .back(30)
                .strafeLeft(6)
                .back(22)
                .addTemporalMarker(4.5, () -> {
                    setDrawerHeight(1500);
                })
                .addTemporalMarker(6, () ->{
                    swoosh1.setPosition(0);
                    swoosh2.setPosition(.2);
                    flop1.setPosition(0.87);
                    flop2.setPosition(0.13);
                })
                .addTemporalMarker(8, () ->{
                    pinch1.setPosition(0.35);
                    pinch2.setPosition(0.9);
                })
                .addTemporalMarker(9.5, () ->{
                    flop1.setPosition(0.97);
                    flop2.setPosition(0.03);
                    swoosh1.setPosition(.1325);
                    swoosh2.setPosition(0.0675);
                    pinch1.setPosition(0);
                    pinch2.setPosition(1);
                })
                .addTemporalMarker(12, () ->{
                    bringDrawersDown();
                })
                .addTemporalMarker(13.5, () ->{
                    pinch1.setPosition(0.35);
                    pinch2.setPosition(0.9);
                    untoPosition(slide1);
                    slide1.setPower(0);
                })
                .forward(110)
                .addDisplacementMarker(15, () ->{
                    spinTimer.reset();
                    lock_in();
                 })
                .back(110)
                .build();

        TrajectorySequence basicPark = drive.trajectorySequenceBuilder(basic.end())
                .strafeRight(20)
                .back(20)
                .build();

        TrajectorySequence middle_1 = drive.trajectorySequenceBuilder(startPose)
                .forward(26)
                .build();

        TrajectorySequence middle_2 = drive.trajectorySequenceBuilder(middle_1.end())
                .lineToLinearHeading(new Pose2d(24, 40, Math.toRadians(-93)))
                .addTemporalMarker(1, () ->{
                    setDrawerHeight(1500);
                })
                .back(15,
                        SampleMecanumDrive.getVelocityConstraint(9, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence middle_3 = drive.trajectorySequenceBuilder(middle_2.end())
                .addTemporalMarker(0, () ->{
                    bringDrawersDown();
                })
                .addTemporalMarker(3, () ->{
                    pinch1.setPosition(0.35);
                    pinch2.setPosition(0.9);
                    untoPosition(slide1);
                    slide1.setPower(0);
                })
                .lineTo(new Vector2d(65, 0))
                .forward(90)
                .strafeRight(4)
                .build();

        TrajectorySequence middle_4 = drive.trajectorySequenceBuilder(middle_3.end())
                .turn(Math.toRadians(-10))
                .lineToConstantHeading(new Vector2d(65, 0))
                .lineTo(new Vector2d(28, 40))
                .addTemporalMarker(2, () ->{
                    setDrawerHeight(1500);
                })
                .addTemporalMarker(1, () -> {
                    if(pinch1.getPosition() == 0){
                        spin.setPower(-1);
                    }

                    if(spinTimer.seconds() > 3){
                        spin.setPower(0);
                    }
                })
                .build();

        TrajectorySequence basic2 = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(0, () ->{
                    pinch1.setPosition(0.35);
                    pinch2.setPosition(0.9);
                })
                .forward(15)
                .build();

        TrajectorySequence basic3 = drive.trajectorySequenceBuilder(basic2.end())
                .back(15)
                .build();

        Globals.ALLIANCE = Globals.Location.BLUE;
        Globals.SIDE = Globals.Location.CLOSE;

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(1920, 1080))
                .addProcessor(propPipeline)
                .build();

        drawerTimer.reset();
        CVTimer.reset();

        while (opModeInInit()) {

            telemetry.addData("Location", propPipeline.getLocation());
            telemetry.addData("leftZone", propPipeline.left.toString());
            telemetry.addData("centerZone", propPipeline.center.toString());
            telemetry.update();
        }

        setStrikePosition();

        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {
            spinTimer.reset();


            currentState = State.TRAJECTORY3;
            count++;

            while (opModeIsActive()) {

                telemetry.addData("Prop Position", strikePos);
                telemetry.addData("state:", currentState);
                telemetry.addData("bussin: " , drive.isBusy());
                telemetry.update();

                switch (currentState) {
                    case START:
                        CVTimer.reset();
                        if(count < 1){

                            drop.setPosition(0.7);

                            portal.stopStreaming();
                            count++;
                            currentState = State.TRAJECTORY;
                        }
                        break;
                    case TRAJECTORY:
                        drive.followTrajectorySequence(middle_1);
                        spinTimer.reset();
                        currentState = State.PIXEL;
                        break;
                    case PIXEL:
                        if(!drive.isBusy()){
                            while(spinTimer.seconds() < 1){
                                spin.setPower(-0.8);
                            }
                            if(spinTimer.seconds() > 1){
                                spin.setPower(0);
                                currentState = State.TRAJECTORY2;
                            }
                        }
                        break;
                    case TRAJECTORY2:
                        drive.followTrajectorySequence(middle_2);
                        currentState = State.DRAWER_FLIP_OUT;
                        break;
                    case DRAWER_FLIP_OUT:
                        if (drawersDone(slide1, slide2)) {
                            swoosh1.setPosition(0);
                            swoosh2.setPosition(.2);
                            flop1.setPosition(0.87);
                            flop2.setPosition(0.13);
                            drawerTimer.reset();
                            currentState = State.RELEASE;
                        }
                        break;
                    case RELEASE:
                        if(drawerTimer.seconds() > 1){
                            pinch1.setPosition(0.35);
                            pinch2.setPosition(0.9);
                            drawerTimer.reset();
                            currentState = State.DRAWER_FLIP_IN;;
                        }
                        break;
                    case DRAWER_FLIP_IN:
                        if (drawerTimer.seconds() > 1.5) {
                            flop1.setPosition(0.97);
                            flop2.setPosition(0.03);
                            swoosh1.setPosition(.1325);
                            swoosh2.setPosition(0.0675);
                            pinch1.setPosition(0);
                            pinch2.setPosition(1);

                            drawerTimer.reset();

                            if(count < 2){
                                count++;
                                currentState = State.TRAJECTORY3;
                            }
                            else{
                                currentState = State.DRAWER_RETRACT;
                            }
                        }
                        break;
                    case DRAWER_RETRACT:
                        if (drawerTimer.seconds() > FLIP_TIME) {
                            bringDrawersDown();

                            drawerTimer.reset();
                            currentState = State.DRAWER_SETTLE;
                        }
                        break;
                    case DRAWER_SETTLE:
                        if ((drawerTimer.seconds() >= 1.5) && drawersDone(slide1, slide1)) {
                            pinch1.setPosition(0.35);
                            pinch2.setPosition(0.9);
                            untoPosition(slide1);
                            slide1.setPower(0);
                            drawerTimer.reset();
                        }
                        currentState = State.START;
                        break;
                    case TRAJECTORY3:
                        /*
                        if (drawerTimer.seconds() > FLIP_TIME) {
                            drive.followTrajectorySequence(middle_3);
                        }

                         */

                        drive.followTrajectorySequence(basic2);
                        spinTimer.reset();
                        if(!drive.isBusy() && drawerTimer.seconds() > 3){
                            spinTimer.reset();
                            currentState = State.LOCK_IN;
                        }
                        break;
                    case LOCK_IN:
                        if(!drive.isBusy()){
                            drop.setPosition(0.31);

                            while(spinTimer.seconds() < 3){
                                spin.setPower(1);
                            }

                            if(spinTimer.seconds() > 2){
                                drop.setPosition(.29);
                            }

                            if(spinTimer.seconds() > 1.5){
                                pinch1.setPosition(0);
                                pinch2.setPosition(1);
                            }
                        }

                        if(spinTimer.seconds() > 3.5){
                            spin.setPower(-1);
                            currentState = State.TRAJECTORY4;
                        }

                        break;
                    case TRAJECTORY4:
                        /*
                        drive.followTrajectorySequence(middle_4);
                        currentState = State.DRAWER_FLIP_OUT;
                         */
                        drive.followTrajectorySequence(basic3);
                        currentState = State.START;
                        break;
                    default:
                        currentState = State.START;
                }

            }
        }
    }

    public void setStrikePosition(){
        if (propPipeline.getLocation() == Globals.Location.LEFT) {
            strikePos = StrikePosition.LEFT;
        }
        else if (propPipeline.getLocation() == Globals.Location.RIGHT) {
            strikePos = StrikePosition.RIGHT;
        }
        else {
            strikePos = StrikePosition.MIDDLE;
        }
    }

    public void bringDrawersDown(){
        while(!magnetic.isPressed() && !magnetic2.isPressed()){
            slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slide1.setPower(-1);
            //slide2.setPower(-1);

            if(magnetic.isPressed() || magnetic2.isPressed()){
                break;
            }
        }
    }

    public void reset(){
        slide1.setPower(0);
        slide2.setPower(0);

        slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slide1.setTargetPosition(0);
        slide2.setTargetPosition(0);

        slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setDrawerHeight(int h){
        height = h;
        movevertically(slide1, h, 1);
    }

    public void waitforDrawers(DcMotor george, DcMotor BobbyLocks) {
        while (!(george.getCurrentPosition() > george.getTargetPosition() - errorBound && george.getCurrentPosition() < george.getTargetPosition() + errorBound) &&
                (BobbyLocks.getCurrentPosition() > BobbyLocks.getTargetPosition() - errorBound && BobbyLocks.getCurrentPosition() < BobbyLocks.getTargetPosition() + errorBound));
    }

    public boolean drawersDone(DcMotor george, DcMotor BobbyLocks) {
        return ((george.getCurrentPosition() > george.getTargetPosition() - errorBound && george.getCurrentPosition() < george.getTargetPosition() + errorBound) &&
                (BobbyLocks.getCurrentPosition() > BobbyLocks.getTargetPosition() - errorBound && BobbyLocks.getCurrentPosition() < BobbyLocks.getTargetPosition() + errorBound));
    }

    public void movevertically(DcMotorEx lipsey, int position, double power) {
        untoPosition(lipsey);
        runtoPosition(lipsey);
        lipsey.setTargetPosition(lipsey.getCurrentPosition() + position);
        lipsey.setPower(power);
    }

    public void runtoPosition(DcMotorEx John) {
        John.setTargetPosition(0);
        John.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        John.setPower(0);
    }

    public void untoPosition(DcMotorEx Neil) {
        Neil.setPower(0);
        Neil.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void stall(DcMotorEx DcMotar) {
        DcMotar.setZeroPowerBehavior(brake);
        DcMotar.setPower(0);
    }
    public void lock_in(){
        drop.setPosition(0.35);

        while(spinTimer.seconds() < 2){
            spin.setPower(1);
        }

        pinch1.setPosition(0);
        pinch2.setPosition(1);

        if(pinch1.getPosition() == 0){
            spin.setPower(-1);
        }

        if(spinTimer.seconds() > 5){
            spin.setPower(0);
        }
    }


}