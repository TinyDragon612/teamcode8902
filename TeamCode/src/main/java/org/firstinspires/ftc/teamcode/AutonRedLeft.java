package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import org.firstinspires.ftc.teamcode.vision.PropPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;


@Config
@Autonomous(name = "\uD83D\uDFE5 AutonRedLeft", group = "drive")
public class AutonRedLeft extends LinearOpMode {

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
        CAMERA_SCAN,
        TRAJECTORY,
        PIXEL,
        TRAJECTORY2,
        DOWN,
        SETTLE,
        TRAJECTORY3,
        DRAWER_START,
        DRAWER_FLIP_OUT,
        TRAJECTORY4,
        RELEASE,
        DRAWER_FLIP_IN,
        DRAWER_RETRACT,
        DRAWER_SETTLE
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
    private PropPipeline propPipeline;

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

        TrajectorySequence justPark = drive.trajectorySequenceBuilder(startPose)
                .forward(53)
                .turn(Math.toRadians(-90))
                .forward(120)
                .strafeRight(15)
                .build();

        TrajectorySequence middle_1 = drive.trajectorySequenceBuilder(startPose)
                .forward(26)
                .build();

        TrajectorySequence middle_2 = drive.trajectorySequenceBuilder(middle_1.end())
                .back(15)
                .strafeLeft(15)
                .forward(32)
                .turn(Math.toRadians(-90))
                .build();

        TrajectorySequence middle_3 = drive.trajectorySequenceBuilder(middle_2.end())
                .forward(70)
                .strafeRight(35)
                .turn(Math.toRadians(190))
                .back(30)
                .build();

        TrajectorySequence left_1 = drive.trajectorySequenceBuilder(startPose)
                .forward(26)
                .turn(Math.toRadians(90))
                .build();

        TrajectorySequence left_2 = drive.trajectorySequenceBuilder(left_1.end())
                .strafeRight(19)
                .build();

        TrajectorySequence left_3 = drive.trajectorySequenceBuilder(left_2.end())
                .back(65)
                .strafeLeft(8)
                .back(21)
                .build();

        TrajectorySequence right_1 = drive.trajectorySequenceBuilder(startPose)
                .forward(27)
                .turn(Math.toRadians(-90))
                .forward(4)
                .build();

        TrajectorySequence right_2 = drive.trajectorySequenceBuilder(right_1.end())
                .back(4)
                .strafeLeft(14)
                .build();

        TrajectorySequence right_3 = drive.trajectorySequenceBuilder(right_2.end())
                .forward(70)
                .strafeRight(28)
                .turn(Math.toRadians(180))
                .back(25,
                        SampleMecanumDrive.getVelocityConstraint(9, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence good = drive.trajectorySequenceBuilder(startPose)
                .back(20,
                        SampleMecanumDrive.getVelocityConstraint(9, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Globals.ALLIANCE = Globals.Location.RED;
        Globals.SIDE = Globals.Location.FAR;

        propPipeline = new PropPipeline();

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

            currentState = State.START;

            while (opModeIsActive()) {

                telemetry.addData("Prop Position", strikePos);
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
                        if (strikePos == StrikePosition.MIDDLE) {
                            drive.followTrajectorySequence(middle_1);
                        } else if (strikePos == StrikePosition.LEFT) {
                            drive.followTrajectorySequence(left_1);
                        } else {
                            drive.followTrajectorySequence(right_1);
                        }
                        spinTimer.reset();
                        currentState = State.PIXEL;
                        break;
                    case PIXEL:
                        while(spinTimer.seconds() < 2){
                            spin.setPower(-0.7);
                        }
                        if(spinTimer.seconds() > 2){
                            spin.setPower(0);
                            currentState = State.TRAJECTORY2;
                        }
                        break;
                    case TRAJECTORY2:
                        if (strikePos == StrikePosition.MIDDLE) {
                            drive.followTrajectorySequence(middle_2);
                        } else if (strikePos == StrikePosition.LEFT) {
                            drive.followTrajectorySequence(left_2);
                        } else {
                            drive.followTrajectorySequence(right_2);
                        }
                        currentState = State.TRAJECTORY3;
                        break;
                    case TRAJECTORY3:
                        if (strikePos == StrikePosition.MIDDLE) {
                            drive.followTrajectorySequence(middle_3);
                        } else if (strikePos == StrikePosition.LEFT) {
                            drive.followTrajectorySequence(left_3);
                        } else {
                            drive.followTrajectorySequence(right_3);
                        }
                        currentState = State.DRAWER_START;
                        break;
                    case DRAWER_START:
                        if (!drive.isBusy()) {
                            setDrawerHeight(1500);
                            currentState = State.DRAWER_FLIP_OUT;
                        }
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
                    case TRAJECTORY4:
                        if(flop1.getPosition()== 0.87) {
                            drive.setPoseEstimate(startPose);
                            drive.followTrajectorySequence(good);
                        }
                        currentState = State.RELEASE;
                        break;
                    case RELEASE:
                        if(drawerTimer.seconds() > 2){
                            pinch1.setPosition(0.35);
                            pinch2.setPosition(0.9);
                            drawerTimer.reset();
                            currentState = State.DRAWER_FLIP_IN;
                        }
                        break;
                    case DRAWER_FLIP_IN:
                        if (drawerTimer.seconds() > 1.5) {
                            flop1.setPosition(0.98);
                            flop2.setPosition(0.02);
                            swoosh1.setPosition(.133);
                            swoosh2.setPosition(0.067);
                            pinch1.setPosition(0);
                            pinch2.setPosition(1);

                            drawerTimer.reset();
                            currentState = State.DRAWER_RETRACT;
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
                            reset();

                            drawerTimer.reset();
                        }
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


}