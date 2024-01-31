package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import org.firstinspires.ftc.teamcode.vision.BluePropThreshold;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

//@Disabled
@Config
@Autonomous(group = "drive")
public class AutonBlueJustPark extends LinearOpMode {
    private DcMotorEx blueboi1, blueboi2;
    private Servo littleflip;

    private Servo bigflip1, bigflip2;

    private int errorBound = 60;

    int height;
    private ServoImplEx poolNoodle;

    ElapsedTime servoTimer = new ElapsedTime();
    ElapsedTime drawerTimer = new ElapsedTime();
    public  ElapsedTime CVTimer = new ElapsedTime();

    public enum State{
        START,
        TRAJECTORY,
        PIXEL,
        TRAJECTORY2,
        DOWN,
        SETTLE,
        TRAJECTORY3,
        DRAWER_SETTLE
    }

    public enum StrikePosition{
        MIDDLE,
        LEFT,
        RIGHT
    }

    State currentState = State.START;
    public StrikePosition strikePos = StrikePosition.MIDDLE;
    public static double CV_RUNTIME = 5;
    public static double FLIP_TIME = 1.5;
    int count;
    private VisionPortal portal;
    private PropPipeline propPipeline;

    @Override
    public void runOpMode() throws InterruptedException {

        blueboi1 = hardwareMap.get(DcMotorEx.class, "blueboi1");
        blueboi2 = hardwareMap.get(DcMotorEx.class, "blueboi2");

        littleflip = hardwareMap.get(Servo.class, "littleflip");
        bigflip1 = hardwareMap.get(Servo.class, "bigflip1");
        bigflip2 = hardwareMap.get(Servo.class, "bigflip2");

        poolNoodle = hardwareMap.get(ServoImplEx.class, "poolNoodle");

        blueboi1.setDirection(DcMotorEx.Direction.REVERSE);
        blueboi2.setDirection(DcMotorEx.Direction.FORWARD);

        blueboi1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blueboi2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        blueboi1.setTargetPosition(0);
        blueboi2.setTargetPosition(0);

        blueboi1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blueboi2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        bigflip1.setPosition(0.5);
        bigflip2.setPosition(0.5);

        littleflip.setPosition(0.7);

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
                .forward(29)
                .build();

        TrajectorySequence middle_2 = drive.trajectorySequenceBuilder(middle_1.end())
                .back(15)
                .strafeRight(20)
                .forward(35)
                .turn(Math.toRadians(80))
                .build();

        TrajectorySequence middle_3 = drive.trajectorySequenceBuilder(middle_2.end())
                .forward(90)
                .strafeLeft(40)
                .turn(Math.toRadians(200)) //230
                .back(10,
                        SampleMecanumDrive.getVelocityConstraint(9, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence right_1 = drive.trajectorySequenceBuilder(startPose) //DONE
                .forward(28)
                .turn(Math.toRadians(-80))
                .forward(3) //4
                .build();

        TrajectorySequence right_2 = drive.trajectorySequenceBuilder(right_1.end()) //DONE
                .back(4)
                .strafeLeft(30)
                .turn(Math.toRadians(180))
                .build();

        TrajectorySequence right_3 = drive.trajectorySequenceBuilder(right_2.end()) //DONE
                .forward(70)
                .strafeLeft(17) //15
                .turn(Math.toRadians(190)) //220
                .back(19,
                        SampleMecanumDrive.getVelocityConstraint(9, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence left_1 = drive.trajectorySequenceBuilder(startPose)
                .forward(26)
                .turn(Math.toRadians(80))
                .forward(6)
                .build();

        TrajectorySequence left_2 = drive.trajectorySequenceBuilder(left_1.end())
                .back(6)
                .strafeRight(30)
                .build();

        TrajectorySequence left_3 = drive.trajectorySequenceBuilder(left_2.end())
                .forward(75)
                .strafeLeft(52)
                .turn(Math.toRadians(190)) //230
                .back(10,
                        SampleMecanumDrive.getVelocityConstraint(9, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence good = drive.trajectorySequenceBuilder(startPose)
                .back(30,
                        SampleMecanumDrive.getVelocityConstraint(9, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Globals.ALLIANCE = Globals.Location.BLUE;
        Globals.SIDE = Globals.Location.FAR;

        propPipeline = new PropPipeline();

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(1920, 1080))
                .addProcessor(propPipeline)
                .build();

        CVTimer.reset();
        drawerTimer.reset();

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

                telemetry.addData("Prop Position", propPipeline.getLocation());
                telemetry.update();

                switch (currentState) {
                    case START:
                        CVTimer.reset();
                        if(count < 1){
                            littleflip.setPosition(0.7);
                            setDrawerHeight(100);
                            bigflip1.setPosition(0.5);
                            bigflip1.setPosition(0.5);

                            while (CVTimer.seconds() < CV_RUNTIME) {
                                setStrikePosition();
                            }

                            portal.stopStreaming();
                            count++;
                            currentState = State.TRAJECTORY;
                        }
                        break;
                    case TRAJECTORY:

                        if (strikePos ==  StrikePosition.MIDDLE) {
                            drive.followTrajectorySequence(middle_1);
                        } else if (strikePos ==  StrikePosition.LEFT) {
                            drive.followTrajectorySequence(left_1);
                        } else {
                            drive.followTrajectorySequence(right_1);
                        }

                        currentState = State.PIXEL;
                        break;
                    case PIXEL:
                        bringDrawersDown();
                        littleflip.setPosition(0.5);
                        waitforDrawers(blueboi1, blueboi2);
                        setDrawerHeight(100);
                        littleflip.setPosition(0.7);
                        currentState = State.TRAJECTORY2;
                        break;
                    case TRAJECTORY2:
                        //drive.setPoseEstimate(startPose);
                        //drive.followTrajectorySequence(good);

                        if (strikePos == StrikePosition.MIDDLE) {
                            drive.followTrajectorySequence(middle_2);
                        } else if (strikePos == StrikePosition.LEFT) {
                            drive.followTrajectorySequence(left_2);
                        } else {
                            drive.followTrajectorySequence(right_2);
                        }

                        currentState = State.DOWN;
                        break;
                    case DOWN:
                        if(!drive.isBusy()){
                            poolNoodleDown();
                            bringDrawersDown();
                        }
                        currentState = State.SETTLE;
                        break;
                    case SETTLE:
                        if (drawersDone(blueboi1, blueboi2)) {
                            untoPosition(blueboi1);
                            untoPosition(blueboi2);
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

                        currentState = State.DRAWER_SETTLE;
                        break;
                    case DRAWER_SETTLE:
                        if ((drawerTimer.seconds() >= 1.5) && drawersDone(blueboi1, blueboi2)) {
                            littleflip.setPosition(0.5);
                            untoPosition(blueboi1);
                            untoPosition(blueboi2);

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
            strikePos = AutonBlueJustPark.StrikePosition.LEFT;
        }
        else if (propPipeline.getLocation() == Globals.Location.RIGHT) {
            strikePos = AutonBlueJustPark.StrikePosition.RIGHT;
        }
        else {
            strikePos = AutonBlueJustPark.StrikePosition.MIDDLE;
        }
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

    private void poolNoodleDown(){
        servoTimer.reset();
        poolNoodle.setPosition(0.7);

        if(servoTimer.seconds() > 20){
            poolNoodle.setPwmDisable();
        }
    }

    public void bringDrawersDown(){
        movevertically(blueboi1, -height-50, 1);
        movevertically(blueboi2, -height-50, 1);
    }

    public void setDrawerHeight(int h){
        height = h;
        movevertically(blueboi1, h, 1);
        movevertically(blueboi2, h, 1);
    }

    public boolean waitforDrawers(DcMotor george, DcMotor BobbyLocks) {
        return ((george.getCurrentPosition() > george.getTargetPosition() - errorBound && george.getCurrentPosition() < george.getTargetPosition() + errorBound) &&
                (BobbyLocks.getCurrentPosition() > BobbyLocks.getTargetPosition() - errorBound && BobbyLocks.getCurrentPosition() < BobbyLocks.getTargetPosition() + errorBound));
    }


}