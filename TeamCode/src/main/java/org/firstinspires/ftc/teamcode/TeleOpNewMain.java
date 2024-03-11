package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PWMOutputImplEx;


@TeleOp(name= "TeleOpNewMain", group="Linear Opmode")
//@Disabled
public class TeleOpNewMain extends LinearOpMode {

    // Declare OpMode members.
    private DcMotorEx.ZeroPowerBehavior brake = DcMotorEx.ZeroPowerBehavior.BRAKE;
    private DcMotorEx.ZeroPowerBehavior floatt = DcMotorEx.ZeroPowerBehavior.FLOAT;

    private DcMotorEx topRight, bottomRight, topLeft, bottomLeft; //wheels

    private DcMotorEx slide1, slide2; //drawers

    private TouchSensor magnetic, magnetic2;

    private ServoImplEx swoosh1, swoosh2, flop1, flop2, pinch1, pinch2, drop, launcher;

    private DcMotorEx spin;

    private int errorBound = 60;
    int height;
    final double FLIP_TIME = 1.5;

    public enum driveState{
        FORWARD,
        BACKWARD
    };

    public enum state {
        DRAWER_START,
        DRAWER_FLIP_IN,
        DRAWER_FLIP_OUT,
        DRAWER_RETRACT,
        DRAWER_SETTLE,
        CUSTOM

    };

    state drawerState = state.DRAWER_START;
    driveState drive = driveState.FORWARD;

    ElapsedTime drawerTimer = new ElapsedTime();
    ElapsedTime servoTimer = new ElapsedTime();

    ElapsedTime slideTimer = new ElapsedTime();

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

        telemetry.update();

        topRight.setDirection(DcMotorEx.Direction.FORWARD);
        bottomRight.setDirection(DcMotorEx.Direction.REVERSE);
        topLeft.setDirection(DcMotorEx.Direction.REVERSE);
        bottomLeft.setDirection(DcMotorEx.Direction.REVERSE);

        slide1.setDirection(DcMotorEx.Direction.FORWARD);
        slide2.setDirection(DcMotorEx.Direction.REVERSE);

        slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slide1.setTargetPosition(0);
        slide2.setTargetPosition(0);

        slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        /*
        slide1.setZeroPowerBehavior(brake);
        slide2.setZeroPowerBehavior(brake);

         */

        swoosh1.setPosition(.1325);
        swoosh2.setPosition(0.0675);

        flop1.setPosition(0.97);
        flop2.setPosition(0.03);

        drop.setPosition(0.9);

        pinch1.setPosition(0.35);
        pinch2.setPosition(0.9);

        launcher.setPosition(0.6);

        spin.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spin.setDirection(DcMotorEx.Direction.REVERSE);

        servoTimer.reset();
        telemetry.update();
        drawerState = state.DRAWER_START;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetry.addData("Status", "Running");
                telemetry.addData("Program", "Android Studio");
                telemetry.addData("slide1: ", slide1.getCurrentPosition());
                telemetry.addData("slide2: ", slide2.getCurrentPosition());
                //telemetry.addData("swoosh1: ", swoosh1.getPosition());
                //telemetry.addData("swoosh2: ", swoosh2.getPosition());
                //telemetry.addData("pinch1: ", pinch1.getPosition());
                //telemetry.addData("pinch2: ", pinch2.getPosition());
                telemetry.addData("drop: ", drop.getPosition());
                telemetry.addData("magnetic1", magnetic.isPressed());
                telemetry.addData("magnetic2", magnetic2.isPressed());
                telemetry.addData("State", drawerState);
                telemetry.addData("slide1 run mode: ", slide1.getMode());
                telemetry.update();

                switch (drawerState) {
                    case CUSTOM:

                        slide1.setZeroPowerBehavior(brake);
                        slide2.setZeroPowerBehavior(brake);

                        slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                        //slide2.setPower(-gamepad2.right_stick_y);
                        slide1.setPower(-gamepad2.right_stick_y);

                        if(gamepad2.left_bumper){
                            slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            drawerState = state.DRAWER_FLIP_IN;
                        }

                        break;
                    case DRAWER_START:
                        if (gamepad2.y) {
                            setDrawerHeight(3000);
                            drawerState = state.DRAWER_FLIP_OUT;
                        } else if (gamepad2.b) {
                            setDrawerHeight(2250);
                            drawerState = state.DRAWER_FLIP_OUT;
                        } else if (gamepad2.a) {
                            setDrawerHeight(1500);
                            drawerState = state.DRAWER_FLIP_OUT;
                        }
                        break;
                    case DRAWER_FLIP_OUT:
                        if (drawersDone(slide1, slide1)) {

                            swoosh1.setPosition(0);
                            swoosh2.setPosition(.2);
                            flop1.setPosition(0.87);
                            flop2.setPosition(0.13);

                            drawerState = state.DRAWER_FLIP_IN;
                        }
                        break;

                    case DRAWER_FLIP_IN:

                        if(gamepad2.right_bumper){
                            slide1.setZeroPowerBehavior(brake);
                            slide2.setZeroPowerBehavior(brake);
                            drawerState = state.CUSTOM;
                        }

                        if (gamepad2.x) {
                            flop1.setPosition(0.97);
                            flop2.setPosition(0.03);
                            swoosh1.setPosition(.1325);
                            swoosh2.setPosition(0.0675);
                            pinch1.setPosition(0);
                            pinch2.setPosition(1);

                            drawerTimer.reset();
                            drawerState = state.DRAWER_RETRACT;
                        }

                        if (gamepad2.y) {
                            setDrawerHeight(3000);
                        } else if (gamepad2.b) {
                            setDrawerHeight(1500);
                        } else if (gamepad2.a) {
                            setDrawerHeight(1000);
                        }

                        break;
                    case DRAWER_RETRACT:
                        if (drawerTimer.seconds() >= FLIP_TIME) {
                            bringDrawersDown();
                            drawerTimer.reset();
                            drawerState = state.DRAWER_SETTLE;
                        }
                        break;
                    case DRAWER_SETTLE:
                        if (magnetic.isPressed() || magnetic2.isPressed()) {
                            pinch1.setPosition(0.35);
                            pinch2.setPosition(0.9);
                            untoPosition(slide1);
                            //untoPosition(slide2);
                            reset();
                            drawerState = state.DRAWER_START;
                        }
                        break;
                    default:
                        drawerState = state.DRAWER_START;
                }

                spin.setPower(gamepad2.left_stick_y);

                //GAMEPAD1 CONTROLS
                if(gamepad1.y){
                    setDrawerHeight(2000);
                }

                if(gamepad1.a) {
                    movevertically(slide1, -500, 0.5);
                    //movevertically(slide2, -500, 0.5);
                }

                if(gamepad1.dpad_up){
                    drop.setPwmEnable();
                    drop.setPosition(0.9);
                }

                if(gamepad1.dpad_down){
                    drop.setPosition(0);

                    if (servoTimer.seconds() > 20) {
                        drop.setPwmDisable();
                    }
                }

                if(gamepad1.touchpad){
                    launcher.setPosition(0);
                }

                if(gamepad1.left_bumper){
                    pinch1.setPosition(0.35);
                }

                if(gamepad1.left_trigger > 0.5){
                    pinch2.setPosition(0.9);
                }

                if(gamepad1.right_bumper){
                    pinch1.setPosition(0);
                }

                if(gamepad1.right_trigger > 0.5){
                    pinch2.setPosition(1);
                }

                topRight.setPower(((gamepad1.left_stick_y + gamepad1.left_stick_x)) + (gamepad1.right_stick_x));
                topLeft.setPower(((-gamepad1.left_stick_y + gamepad1.left_stick_x)) + ((gamepad1.right_stick_x)));
                bottomRight.setPower(((gamepad1.left_stick_y + -gamepad1.left_stick_x)) + (gamepad1.right_stick_x));
                bottomLeft.setPower(((-gamepad1.left_stick_y + -gamepad1.left_stick_x)) + (gamepad1.right_stick_x));

            }

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
        //movevertically(slide2, slide1.getCurrentPosition(), 1);

        /*
        slideTimer.reset();

        movevertically(slide2, h, 1);
        if(slideTimer.milliseconds() > 500){
            movevertically(slide1, h, 1);
        }

         */
    }

    public void waitforDrawer(DcMotor george) {
        while(!(george.getCurrentPosition() > george.getTargetPosition() - errorBound && george.getCurrentPosition() < george.getTargetPosition() + errorBound));
    }

    public boolean waitforDrawers(DcMotor george, DcMotor BobbyLocks) {
        return ((george.getCurrentPosition() > george.getTargetPosition() - errorBound && george.getCurrentPosition() < george.getTargetPosition() + errorBound) &&
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