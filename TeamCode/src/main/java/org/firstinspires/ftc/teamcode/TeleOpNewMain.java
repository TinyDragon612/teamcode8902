package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.TouchSensor;


@TeleOp(name= "TeleOpNewMain", group="Linear Opmode")
//@Disabled
public class TeleOpNewMain extends LinearOpMode {

    // Declare OpMode members.
    private DcMotorEx.ZeroPowerBehavior brake = DcMotorEx.ZeroPowerBehavior.BRAKE;
    private DcMotorEx.ZeroPowerBehavior floatt = DcMotorEx.ZeroPowerBehavior.FLOAT;

    private DcMotorEx topRight, bottomRight, topLeft, bottomLeft; //wheels

    private DcMotorEx slide1, slide2; //drawers

    private TouchSensor magnetic, magnetic2;

    private ServoImplEx swoosh1, swoosh2, flop1, flop2, pinch1, pinch2;

    private DcMotorEx spin;

    private int errorBound = 60;
    int height;

    final double FLIP_TIME = 0.75;

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

        slide1.setZeroPowerBehavior(brake);
        slide2.setZeroPowerBehavior(brake);

        swoosh1.setPosition(0);
        swoosh2.setPosition(.2);

        spin.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spin.setDirection(DcMotorEx.Direction.REVERSE);

        servoTimer.reset();
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                slide1.setZeroPowerBehavior(brake);
                slide2.setZeroPowerBehavior(brake);

                telemetry.addData("Status", "Running");
                telemetry.addData("Program", "Android Studio");
                telemetry.addData("slide1: ", slide1.getCurrentPosition());
                telemetry.addData("slide2: ", slide2.getCurrentPosition());
                //telemetry.addData("swoosh1: ", swoosh1.getPosition());
                //telemetry.addData("swoosh2: ", swoosh2.getPosition());
                //telemetry.addData("pinch1: ", pinch1.getPosition());
                //telemetry.addData("pinch2: ", pinch2.getPosition());
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

                        slide2.setPower(-gamepad2.right_stick_y);
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
                            setDrawerHeight(1500);
                            drawerState = state.DRAWER_FLIP_OUT;
                        } else if (gamepad2.a) {
                            setDrawerHeight(1000);
                            drawerState = state.DRAWER_FLIP_OUT;
                        }
                        break;
                    case DRAWER_FLIP_OUT:
                        if (waitforDrawers(slide1, slide2)) {

                            //swoosh1.setPosition(.1325);
                            //swoosh2.setPosition(0.0675);
                            //flop1.setPosition(x)
                            //flop2.setPosition(x)

                        }
                        drawerState = state.DRAWER_FLIP_IN;
                        break;

                    case DRAWER_FLIP_IN:

                        if(gamepad2.right_bumper){
                            slide1.setZeroPowerBehavior(brake);
                            slide2.setZeroPowerBehavior(brake);
                            drawerState = state.CUSTOM;
                        }

                        if (gamepad2.x) {
                            //swoosh1.setPosition(0);
                            //swoosh2.setPosition(.2);
                            //flop1.setPosition(x)
                            //flop2.setPosition(x)

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
                            untoPosition(slide1);
                            untoPosition(slide2);
                            reset();
                            drawerState = state.DRAWER_START;
                        }
                        break;
                    default:
                        drawerState = state.DRAWER_START;
                }


                /*
                if (gamepad2.x) {
                    bringDrawersDown();

                    if(magnetic.isPressed() || magnetic2.isPressed()){
                        reset();
                    }
                }

                 */

                if(gamepad2.dpad_up){
                    setDrawerHeight(2000);
                }

                if(gamepad2.dpad_down){
                    movevertically(slide1, -500, 0.5);
                    movevertically(slide2, -500, 0.5);
                }

                spin.setPower(gamepad2.left_stick_y);

                //GAMEPAD1 CONTROLS

                if(gamepad1.dpad_up){
                    swoosh1.setPosition(.1325);
                    swoosh2.setPosition(0.0675);
                }

                if(gamepad1.dpad_down){
                    swoosh1.setPosition(0);
                    swoosh2.setPosition(.2);
                }

                if(gamepad1.dpad_left){
                    flop1.setPosition(0);
                    flop2.setPosition(1);
                }

                if(gamepad1.dpad_right){
                    flop1.setPosition(1);
                    flop2.setPosition(0);
                }

                if(gamepad1.left_bumper){
                    pinch1.setPosition(0);
                    pinch2.setPosition(0);
                }

                if(gamepad1.right_bumper){
                    pinch1.setPosition(0.5);
                    pinch2.setPosition(0.5);
                }

                /*
                YAY CONFUSING TEH DRIVER

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

                        if(gamepad1.a){
                            drive = driveState.BACKWARD;
                        }
                        break;

                    case BACKWARD:

                        topRight.setDirection(DcMotorEx.Direction.REVERSE);
                        bottomRight.setDirection(DcMotorEx.Direction.FORWARD);
                        topLeft.setDirection(DcMotorEx.Direction.FORWARD);
                        bottomLeft.setDirection(DcMotorEx.Direction.FORWARD);

                        topRight.setPower(((gamepad1.left_stick_y + gamepad1.left_stick_x)) + (-gamepad1.right_stick_x));
                        topLeft.setPower(((-gamepad1.left_stick_y + gamepad1.left_stick_x)) + ((-gamepad1.right_stick_x)));
                        bottomRight.setPower(((gamepad1.left_stick_y + -gamepad1.left_stick_x)) + (-gamepad1.right_stick_x));
                        bottomLeft.setPower(((-gamepad1.left_stick_y + -gamepad1.left_stick_x)) + (-gamepad1.right_stick_x));

                        if(gamepad1.y){
                            drive = driveState.FORWARD;
                        }
                        break;

                }

                 */


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
            slide2.setPower(-1);

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
        movevertically(slide2, h, 1);

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