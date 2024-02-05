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

import org.firstinspires.ftc.teamcode.vision.RedPropThreshold;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;


@Config
@Autonomous(name = "\uD83D\uDFE5 AutonRedRight", group = "drive")
public class AutonRedRight extends LinearOpMode {

    private DcMotorEx blueboi1, blueboi2;
    private Servo littleflip, bigflip1, bigflip2;

    private int errorBound = 60;

    int height;

    private ServoImplEx poolNoodle;

    ElapsedTime CVTimer = new ElapsedTime();
    public ElapsedTime drawerTimer = new ElapsedTime();
    ElapsedTime servoTimer = new ElapsedTime();
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
        TRAJECTORY3,
        RELEASE
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
                .forward(50)
                .build();

        TrajectorySequence basic = drive.trajectorySequenceBuilder(startPose)
                .forward(25)
                .turn(Math.toRadians(90))
                .back(40)
                .waitSeconds(1.5)
                .back(6,
                        SampleMecanumDrive.getVelocityConstraint(9, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence basicPark = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(15)
                .back(20)
                .build();

        TrajectorySequence middle_1 = drive.trajectorySequenceBuilder(startPose)
                .forward(28)
                .build();

        TrajectorySequence middle_2 = drive.trajectorySequenceBuilder(middle_1.end())
                .back(7.5)
                .turn(Math.toRadians(90))
                .back(40)
                .strafeLeft(5)
                .build();

        TrajectorySequence middle_3 = drive.trajectorySequenceBuilder(middle_2.end())
                .forward(5)
                .strafeLeft(30)
                .back(20)
                .build();

        TrajectorySequence left_1 = drive.trajectorySequenceBuilder(startPose) //DONE
                .forward(27)
                .turn(Math.toRadians(80))
                .forward(3)
                .build();

        TrajectorySequence left_2 = drive.trajectorySequenceBuilder(left_1.end()) //DONE
                .back(30)
                .strafeRight(17) //og 1.5
                .back(10,
                        SampleMecanumDrive.getVelocityConstraint(9, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence left_3 = drive.trajectorySequenceBuilder(left_2.end()) //DONE
                .forward(5)
                .strafeLeft(37) //og 40
                .back(20)
                .build();


        TrajectorySequence right_1 = drive.trajectorySequenceBuilder(startPose) //DONE
                .lineToLinearHeading(new Pose2d(35, -17, Math.toRadians(90)))
                .build();

        TrajectorySequence right_2 = drive.trajectorySequenceBuilder(right_1.end()) //DONE
                .back(15)
                .strafeLeft(21)
                .back(10,
                        SampleMecanumDrive.getVelocityConstraint(9, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        /*
        TrajectorySequence right_1 = drive.trajectorySequenceBuilder(startPose) //DONE
                .forward(15)
                .turn(Math.toRadians(-60))
                .forward(5)
                .build();

        TrajectorySequence right_2 = drive.trajectorySequenceBuilder(right_1.end()) //DONE
                .back(5)
                .turn(Math.toRadians(-30))
                .strafeRight(20)
                .strafeLeft(4)
                .turn(Math.toRadians(180))
                .back(35)
                .strafeRight(15)
                .back(30,
                        SampleMecanumDrive.getVelocityConstraint(9, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

         */


        TrajectorySequence right_3 = drive.trajectorySequenceBuilder(right_2.end())
                .forward(5)
                .strafeLeft(20)
                .back(20)
                .build();

        propPipeline = new PropPipeline();

        Globals.ALLIANCE = Globals.Location.RED;
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

            currentState = State.START;

            while (opModeIsActive()) {

                telemetry.addData("Prop Position", strikePos);
                telemetry.update();

                switch (currentState) {
                    case START:
                        CVTimer.reset();
                        if(count < 1){

                            littleflip.setPosition(0.7);
                            setDrawerHeight(100);
                            bigflip1.setPosition(0.5);
                            bigflip1.setPosition(0.5);

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
                        poolNoodleDown();
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
                        if(drawersDone(blueboi1, blueboi2)) {
                            untoPosition(blueboi1);
                            untoPosition(blueboi2);
                        }
                        currentState = State.DRAWER_START;
                        break;
                    case DRAWER_START:
                        if (!drive.isBusy()) {
                            setDrawerHeight(350); //og 410
                            currentState = State.DRAWER_FLIP_OUT;
                        }
                        break;
                    case DRAWER_FLIP_OUT:
                        if (drawersDone(blueboi1, blueboi2)) {
                            bigflip1.setPosition(0.197);
                            bigflip2.setPosition(0.803);
                            drawerTimer.reset();
                            currentState = State.RELEASE;
                        }
                        break;
                    case RELEASE:
                        if(drawerTimer.seconds() > 2){
                            littleflip.setPosition(0.5);
                            drawerTimer.reset();
                            currentState = State.DRAWER_FLIP_IN;
                        }
                        break;
                    case DRAWER_FLIP_IN:
                        if (drawerTimer.seconds() > 1.5) {
                            littleflip.setPosition(0.7);
                            bigflip1.setPosition(0.52);
                            bigflip2.setPosition(0.48);

                            drawerTimer.reset();
                            currentState = State.DRAWER_RETRACT;
                        }
                        break;
                    case DRAWER_RETRACT:
                        if (drawerTimer.seconds() > FLIP_TIME) {
                            height = 500;
                            bringDrawersDown();

                            drawerTimer.reset();
                            currentState = State.DRAWER_SETTLE;
                        }
                        break;
                    case DRAWER_SETTLE:
                        if ((drawerTimer.seconds() >= 1.5) && drawersDone(blueboi1, blueboi2)) {
                            littleflip.setPosition(0.5);
                            untoPosition(blueboi1);
                            untoPosition(blueboi2);

                            drawerTimer.reset();
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
        movevertically(blueboi1, -height-60, 1);
        movevertically(blueboi2, -height-60, 1);
    }

    public void setDrawerHeight(int h){
        height = h;
        movevertically(blueboi1, h, 1);
        movevertically(blueboi2, h, 1);
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

    private void poolNoodleDown(){
        servoTimer.reset();
        poolNoodle.setPosition(0.7);

        if(servoTimer.seconds() > 20){
            poolNoodle.setPwmDisable();
        }
    }

    public void stall(DcMotorEx DcMotar) {
        DcMotar.setZeroPowerBehavior(brake);
        DcMotar.setPower(0);
    }


}