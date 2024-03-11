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


@TeleOp(name= "Test", group="Linear Opmode")
//@Disabled
public class Test extends LinearOpMode {

    // Declare OpMode members.
    private DcMotorEx.ZeroPowerBehavior brake = DcMotorEx.ZeroPowerBehavior.BRAKE;
    private DcMotorEx.ZeroPowerBehavior floatt = DcMotorEx.ZeroPowerBehavior.FLOAT;

    private DcMotorEx topRight, bottomRight, topLeft, bottomLeft; //wheels

    private DcMotorEx slide1, slide2; //drawers

    private TouchSensor magnetic, magnetic2;

    private ServoImplEx swoosh1, swoosh2, flop1, flop2, pinch1, pinch2, drop;

    private DcMotorEx spin;

    private int errorBound = 60;
    int height;
    final double FLIP_TIME = 1.5;

    public enum driveState {
        FORWARD,
        BACKWARD
    }

    ;

    public enum state {
        DRAWER_START,
        DRAWER_FLIP_IN,
        DRAWER_FLIP_OUT,
        DRAWER_RETRACT,
        DRAWER_SETTLE,
        CUSTOM

    }

    ;

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
        spin = hardwareMap.get(DcMotorEx.class, "spin");

        telemetry.update();

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


            }
        }

    }
}