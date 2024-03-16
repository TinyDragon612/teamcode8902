package org.firstinspires.ftc.teamcode.oldCode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Disabled
@Autonomous(group = "drive")
public class AutonHelp extends LinearOpMode {

    private DcMotorEx blueboi1, blueboi2;
    private Servo littleflip;

    private Servo bigflip1, bigflip2;

    private int errorBound = 60;

    public static double RUNTIME = 3.0;

    int height;

    public enum State{
        DRAWER_START,
        DRAWER_FLIP_IN,
        DRAWER_FLIP_OUT,
        DRAWER_RETRACT,
        DRAWER_SETTLE,
        TRAJECTORY,
        TRAJECTORY_PARK
    };

    public enum StrikePosition{
        MIDDLE,
        LEFT,
        RIGHT,
    }

    StrikePosition strikePos = StrikePosition.MIDDLE;

    State currentState;

    ElapsedTime drawerTimer = new ElapsedTime();

    ElapsedTime timer = new ElapsedTime();

    final double FLIP_TIME = 1.0;

    int count;

    private Recognition r;
    double x;

    @Override
    public void runOpMode() throws InterruptedException {
        blueboi1 = hardwareMap.get(DcMotorEx.class, "blueboi1");
        blueboi2 = hardwareMap.get(DcMotorEx.class, "blueboi2");

        littleflip = hardwareMap.get(Servo.class, "littleflip");
        bigflip1 = hardwareMap.get(Servo.class, "bigflip1");
        bigflip2 = hardwareMap.get(Servo.class, "bigflip2");

        blueboi1.setDirection(DcMotorEx.Direction.REVERSE);
        blueboi2.setDirection(DcMotorEx.Direction.FORWARD);

        blueboi1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blueboi2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        blueboi1.setTargetPosition(0);
        blueboi2.setTargetPosition(0);

        blueboi1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blueboi2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        bigflip1.setPosition(0.6);
        bigflip2.setPosition(1);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);

        TrajectorySequence parkFromStart = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(90))
                .forward(53)
                .strafeRight(120)
                .build();

         TrajectorySequence propLeft = drive.trajectorySequenceBuilder(startPose)
                 .lineToConstantHeading(new Vector2d(-21.5,20))
                 .waitSeconds(1)
                 .strafeRight(17)
                 .forward(20)
                 .turn(Math.toRadians(90))
                 .forward(53)
                 .strafeRight(120)
                 .forward(10)
                 .build();

        TrajectorySequence driveSlow = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(33)
                /*
                .forward(14.5,
                        SampleMecanumDrive.getVelocityConstraint(18, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                 */
                .turn(Math.toRadians(-90))
                .strafeLeft(8)
                .strafeRight(8)
                .back(20)
                .strafeLeft(120)
                .forward(15)

                .build();

        TrajectorySequence redRightBoard = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(19.8,23))
                .UNSTABLE_addTemporalMarkerOffset(2, () ->{
                    setDrawerHeight(500);
                    waitforDrawers(blueboi1, blueboi2);
                    littleflip.setPosition(0.5);
                    bringFlipsIn();
                    bringDrawersDown();
                })
                .strafeRight(10)
                .build();

        TrajectorySequence mid = drive.trajectorySequenceBuilder(startPose) //DONE
                .strafeLeft(28)
                .strafeRight(10)
                .back(10)
                .strafeLeft(5)
                .turn(Math.toRadians(360))
                .forward(120)
                .strafeRight(20)
                .build();

        drawerTimer.reset();
        timer.reset();
        telemetry.update();

        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {
            drive.followTrajectorySequence(mid);
        }

        /*
        if (opModeIsActive() && !isStopRequested()) {
            setStrikePosition(r);
            count = 0;
            currentState = State.TRAJECTORY;

            while (opModeIsActive()) {

                switch (currentState) {
                    case TRAJECTORY:
                        if (strikePos == StrikePosition.MIDDLE) {
                            drive.followTrajectorySequence(step1);
                        } else if (strikePos == StrikePosition.LEFT) {
                            drive.followTrajectorySequence(step1);
                        } else {
                            drive.followTrajectorySequence(step1);
                        }
                        currentState = State.DRAWER_START;
                        break;
                    case DRAWER_START:
                        if (!drive.isBusy() && count < 1) {
                            setDrawerHeight(750);
                            count++;
                            currentState = State.DRAWER_FLIP_OUT;
                        }
                        break;
                    case DRAWER_FLIP_OUT:
                        if (drawersDone(blueboi1, blueboi2)) {
                            bigflip1.setPosition(.8);
                            bigflip2.setPosition(0.48);
                            littleflip.setPosition(0.5);

                            drawerTimer.reset();
                            currentState = State.DRAWER_FLIP_IN;
                        }
                        break;
                    case DRAWER_FLIP_IN:
                        if (drawerTimer.seconds() >= 1) {
                            littleflip.setPosition(0.7);
                            bigflip1.setPosition(0.6);
                            bigflip2.setPosition(1);

                            drawerTimer.reset();
                            currentState = State.DRAWER_RETRACT;
                        }
                        break;
                    case DRAWER_RETRACT:
                        if (drawerTimer.seconds() >= FLIP_TIME) {
                            bringDrawersDown();

                            drawerTimer.reset();
                            currentState = State.DRAWER_SETTLE;
                        }
                        break;
                    case DRAWER_SETTLE:
                        if ((drawerTimer.seconds() >= 3) && drawersDone(blueboi1, blueboi2)) {
                            littleflip.setPosition(0.5);
                            untoPosition(blueboi1);
                            untoPosition(blueboi2);

                            timer.reset();
                            timer.wait();
                        }
                        currentState = State.TRAJECTORY_PARK;
                        break;
                    case TRAJECTORY_PARK:
                        drive.setPoseEstimate(startPose);

                        if(strikePos == StrikePosition.MIDDLE){
                            drive.followTrajectorySequence(step2);
                        }
                        else if(strikePos == StrikePosition.LEFT){
                            drive.followTrajectorySequence(step2);
                        }
                        else{
                            drive.followTrajectorySequence(step2);
                        }
                        currentState = State.DRAWER_START;
                        break;
                    default:
                        currentState = State.DRAWER_START;
                }

            }
            }

         */
        }


    public void setStrikePosition(Recognition sally){
        if(sally == null){
            strikePos = StrikePosition.MIDDLE;
        }
        else{
            x = (sally.getLeft() + sally.getRight()) / 2;
            if(x > 1300) {
                strikePos = StrikePosition.RIGHT;
            }
            else{
                strikePos = StrikePosition.LEFT;
            }
        }
    }



    public void bringFlipsIn(){
        littleflip.setPosition(0.7);
        bigflip1.setPosition(0.6);
        bigflip2.setPosition(1);
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

}