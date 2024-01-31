package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad.*;

import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import java.util.Map;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;

import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.util.ElapsedTime.Resolution;
import com.qualcomm.robotcore.hardware.TouchSensor;


@TeleOp(name="LoHicimos", group="Linear Opmode")
//@Disabled
public class TeleOpMain23 extends LinearOpMode {

    // Declare OpMode members.
    private DcMotorEx.ZeroPowerBehavior brake = DcMotorEx.ZeroPowerBehavior.BRAKE;
    private DcMotorEx.ZeroPowerBehavior floatt = DcMotorEx.ZeroPowerBehavior.FLOAT;

    private DcMotorEx topRight, bottomRight, topLeft, bottomLeft; //wheels

    private DcMotorEx alex; //hanger

    private DcMotorEx blueboi1, blueboi2; //drawers

    private DcMotorEx spinyboi; //intake

    private Servo bigflip1, bigflip2; //bring pixel holder out

    private Servo floppy; //release or hold hanger

    private Servo littleflip; // release or hold pixel in holder

    private Servo lettuce; //release or hold intake

    private Servo launcher; // launches paper airplane
    private ServoImplEx poolNoodle;

    private int errorBound = 60;

    boolean weGoin;
    int height;

    final double FLIP_TIME = 0.75;

    public enum State{
        DRAWER_START,
        DRAWER_FLIP_IN,
        DRAWER_FLIP_OUT,
        DRAWER_RETRACT,
        DRAWER_SETTLE,
        BAD

    };

    public enum driveState{
        FORWARD,
        BACKWARD
    };

    State drawerState = State.DRAWER_START;
    driveState drive = driveState.FORWARD;

    ElapsedTime drawerTimer = new ElapsedTime();
    ElapsedTime servoTimer = new ElapsedTime();

    ElapsedTime littleTimer = new ElapsedTime();


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        drawerTimer.reset();

        topRight = hardwareMap.get(DcMotorEx.class, "topRight");
        topLeft = hardwareMap.get(DcMotorEx.class, "topLeft");
        bottomRight = hardwareMap.get(DcMotorEx.class, "bottomRight");
        bottomLeft = hardwareMap.get(DcMotorEx.class, "bottomLeft");
        spinyboi = hardwareMap.get(DcMotorEx.class, "spinyboi");
        alex = hardwareMap.get(DcMotorEx.class, "alex");
        blueboi1 = hardwareMap.get(DcMotorEx.class, "blueboi1");
        blueboi2 = hardwareMap.get(DcMotorEx.class, "blueboi2");
        launcher = hardwareMap.get(Servo.class, "launcher");
        littleflip = hardwareMap.get(Servo.class, "littleflip");
        bigflip1 = hardwareMap.get(Servo.class, "bigflip1");
        bigflip2 = hardwareMap.get(Servo.class, "bigflip2");
        floppy = hardwareMap.get(Servo.class, "floppy");
        lettuce = hardwareMap.get(Servo.class, "lettuce");
        poolNoodle = hardwareMap.get(ServoImplEx.class, "poolNoodle");

        telemetry.update();

        topRight.setDirection(DcMotorEx.Direction.FORWARD);
        bottomRight.setDirection(DcMotorEx.Direction.REVERSE);
        topLeft.setDirection(DcMotorEx.Direction.REVERSE);
        bottomLeft.setDirection(DcMotorEx.Direction.REVERSE);

        blueboi1.setDirection(DcMotorEx.Direction.REVERSE);
        blueboi2.setDirection(DcMotorEx.Direction.FORWARD);
        alex.setDirection(DcMotorEx.Direction.REVERSE);
        spinyboi.setDirection(DcMotorEx.Direction.REVERSE);

        bigflip1.setPosition(0.5);
        bigflip2.setPosition(0.5);

        lettuce.setPosition(0);
        littleflip.setPosition(0.7);
        launcher.setPosition(0.7);

        blueboi1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blueboi2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        blueboi1.setTargetPosition(0);
        blueboi2.setTargetPosition(0);

        blueboi1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        blueboi2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        alex.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        servoTimer.reset();
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetry.addData("Status", "Running");
                telemetry.addData("Program", "Android Studio");
                telemetry.addData("bigflip1:", bigflip1.getPosition());
                telemetry.addData("bigflip2:", bigflip2.getPosition());
                telemetry.addData("Blueboi1: ", blueboi1.getCurrentPosition());
                telemetry.addData("Blueboi2: ", blueboi2.getCurrentPosition());
                telemetry.update();

                switch (drawerState) {
                    case DRAWER_START:

                        if (gamepad2.y) {
                            setDrawerHeight(900);
                            drawerState = State.DRAWER_FLIP_OUT;
                        } else if (gamepad2.b) {
                            setDrawerHeight(600);
                            drawerState = State.DRAWER_FLIP_OUT;
                        } else if (gamepad2.a) {
                            setDrawerHeight(500);
                            drawerState = State.DRAWER_FLIP_OUT;
                        }
                        break;
                    case DRAWER_FLIP_OUT:
                        if (waitforDrawers(blueboi1, blueboi2)) {

                            bigflip1.setPosition(0.195); //.19, .20
                            bigflip2.setPosition(0.805); //0.81, .8

                            drawerState = State.DRAWER_FLIP_IN;
                        }
                        break;

                    case DRAWER_FLIP_IN:

                        if (gamepad2.x) {
                            bigflip1.setPosition(0.5);
                            bigflip2.setPosition(0.5);
                            littleflip.setPosition(0.7);

                            drawerTimer.reset();
                            drawerState = State.DRAWER_RETRACT;
                        }
                        break;
                    case DRAWER_RETRACT:
                        if (drawerTimer.seconds() >= FLIP_TIME) {
                            bringDrawersDown();

                            drawerTimer.reset();
                            drawerState = State.DRAWER_SETTLE;
                        }
                        break;
                    case DRAWER_SETTLE:
                        if ((drawerTimer.seconds() >= 1) && waitforDrawers(blueboi1, blueboi2)) {
                            littleflip.setPosition(0.5);
                            untoPosition(blueboi1);
                            untoPosition(blueboi2);

                            drawerState = State.DRAWER_START;
                        }
                        break;
                    case BAD:
                        emergency();
                        drawerState = State.DRAWER_START;
                        break;
                    default:
                        drawerState = State.DRAWER_START;
                }

                //GAMEPAD1 CONTROLS

                /*

                switch(drive){
                    case FORWARD:

                        topRight.setDirection(DcMotorEx.Direction.FORWARD);
                        bottomRight.setDirection(DcMotorEx.Direction.REVERSE);
                        topLeft.setDirection(DcMotorEx.Direction.REVERSE);
                        bottomLeft.setDirection(DcMotorEx.Direction.REVERSE);

                        topRight.setPower(((gamepad1.left_stick_y + gamepad1.left_stick_x)) + (gamepad1.right_stick_x));
                        topLeft.setPower(((-gamepad1.left_stick_y + gamepad1.left_stick_x)) + ((gamepad1.right_stick_x)));
                        bottomRight.setPower(((gamepad1.left_stick_y + -gamepad1.left_stick_x)) + (gamepad1.right_stick_x));
                        bottomLeft.setPower(((-gamepad1.left_stick_y + -gamepad1.left_stick_x)) + (gamepad1.right_stick_x));

                    case BACKWARD:

                        topRight.setDirection(DcMotorEx.Direction.REVERSE);
                        bottomRight.setDirection(DcMotorEx.Direction.FORWARD);
                        topLeft.setDirection(DcMotorEx.Direction.FORWARD);
                        bottomLeft.setDirection(DcMotorEx.Direction.FORWARD);

                        topRight.setPower(((gamepad1.left_stick_y + gamepad1.left_stick_x)) + (-gamepad1.right_stick_x));
                        topLeft.setPower(((-gamepad1.left_stick_y + gamepad1.left_stick_x)) + ((-gamepad1.right_stick_x)));
                        bottomRight.setPower(((gamepad1.left_stick_y + -gamepad1.left_stick_x)) + (-gamepad1.right_stick_x));
                        bottomLeft.setPower(((-gamepad1.left_stick_y + -gamepad1.left_stick_x)) + (-gamepad1.right_stick_x));

                }

                 */

                topRight.setPower(((gamepad1.left_stick_y + gamepad1.left_stick_x)) + (gamepad1.right_stick_x));
                topLeft.setPower(((-gamepad1.left_stick_y + gamepad1.left_stick_x)) + ((gamepad1.right_stick_x)));
                bottomRight.setPower(((gamepad1.left_stick_y + -gamepad1.left_stick_x)) + (gamepad1.right_stick_x));
                bottomLeft.setPower(((-gamepad1.left_stick_y + -gamepad1.left_stick_x)) + (gamepad1.right_stick_x));


                if (gamepad1.dpad_down) {
                    lettuce.setPosition(0.55);
                }

                if (gamepad1.dpad_up) {
                    launcher.setPosition(0);
                }

                if (gamepad1.dpad_left) {
                    littleflip.setPosition(0.5);
                    sleep(10);
                    littleflip.setPosition(0.7);
                }

                if (gamepad1.right_bumper) {
                    littleflip.setPosition(0.7);
                }

                if (gamepad1.left_bumper) {
                    littleflip.setPosition(0.5);
                }

                if (gamepad1.x) {
                    bigflip1.setPosition(0.5);
                    bigflip2.setPosition(0.5);
                }

                if (gamepad1.a) {
                    drawerState = State.BAD;
                }

                //GAMEPAD2 CONTROLS
                spinyboi.setPower(gamepad2.left_stick_y); //send power to motor to spin intake
                alex.setPower(gamepad2.right_stick_y); //send power to motor to lift/hang bot

                if (gamepad2.right_bumper) {
                    floppy.setPosition(0.55); //release the hanger
                }

                if (gamepad2.left_bumper) {
                    lettuce.setPosition(0.55); //lower the intake
                }


                if (gamepad2.dpad_up) {
                    bigflip1.setPosition(0.5);
                    bigflip2.setPosition(0.5);

                    //flips in
                }

                if (gamepad2.dpad_down) {

                    bigflip1.setPosition(0.2);
                    bigflip2.setPosition(0.8);

                    if (gamepad2.dpad_left) {
                        servoTimer.reset();
                        poolNoodle.setPosition(0.7);

                        if (servoTimer.seconds() > 20) {
                            poolNoodle.setPwmDisable();
                        }
                    }

                    if (gamepad2.dpad_right) {

                    }


                }


            }

        }
    }

    public void bringDrawersDown(){
        movevertically(blueboi1, -height-50, 1);
        movevertically(blueboi2, -height-50, 1);
    }

    public void emergency(){
            setDrawerHeight(-600);
            waitforDrawers(blueboi1, blueboi2);

            blueboi1.setPower(0);
            blueboi2.setPower(0);

            blueboi1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            blueboi2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            blueboi1.setTargetPosition(0);
            blueboi2.setTargetPosition(0);

            blueboi1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            blueboi2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setDrawerHeight(int h){
        height = h;
        movevertically(blueboi1, h, 1);
        movevertically(blueboi2, h, 1);
    }

    public void waitforDrawer(DcMotor george) {
        while(!(george.getCurrentPosition() > george.getTargetPosition() - errorBound && george.getCurrentPosition() < george.getTargetPosition() + errorBound));
    }

    public boolean waitforDrawers(DcMotor george, DcMotor BobbyLocks) {
        return ((george.getCurrentPosition() > george.getTargetPosition() - errorBound && george.getCurrentPosition() < george.getTargetPosition() + errorBound) &&
                (BobbyLocks.getCurrentPosition() > BobbyLocks.getTargetPosition() - errorBound && BobbyLocks.getCurrentPosition() < BobbyLocks.getTargetPosition() + errorBound));
    }

    public void movevertically(DcMotorEx lipsey, int position, double power) {
        untoPosition(lipsey);
        runtoPosition(lipsey);
        lipsey.setTargetPosition(lipsey.getCurrentPosition() + position);
        lipsey.setPower(power);

    }

    public void nostall(DcMotorEx Harry) {
        Harry.setZeroPowerBehavior(floatt);
        Harry.setPower(0);
    }

    public void stall(DcMotorEx DcMotar) {
        DcMotar.setZeroPowerBehavior(brake);
        DcMotar.setPower(0);
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