package org.firstinspires.ftc.teamcode.oldCode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Disabled
@Autonomous(name="IterativeAuton", group = "drive")
public class AutonBad extends OpMode{

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
        DRAWER_SETTLE

    };

    State drawerState;

    ElapsedTime drawerTimer = new ElapsedTime();

    ElapsedTime timer = new ElapsedTime();

    final double FLIP_TIME = 1.0;

    int count;

    @Override
    public void init() {
        blueboi1 = hardwareMap.get(DcMotorEx.class, "blueboi1");
        blueboi2 = hardwareMap.get(DcMotorEx.class, "blueboi2");

        littleflip = hardwareMap.get(Servo.class, "littleflip");
        bigflip1 = hardwareMap.get(Servo.class, "bigflip1");
        bigflip2 = hardwareMap.get(Servo.class, "bigflip2");

        telemetry.update();

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

        drawerState = State.DRAWER_START;

        drawerTimer.reset();
        timer.reset();

        telemetry.update();
    }


        public void init_loop(){
            telemetry.addData("BlueBoi1", blueboi1.getCurrentPosition());
            telemetry.update();
        }

        public void start() {
            telemetry.addData("BlueBoi1", blueboi1.getCurrentPosition());
            telemetry.update();

        }

        public void loop(){

                switch (drawerState) {
                    case DRAWER_START:
                        if((timer.seconds() > 3) && (count < 1)){
                            setDrawerHeight(750);
                            count++;
                            drawerState = State.DRAWER_FLIP_OUT;
                        }
                        break;
                    case DRAWER_FLIP_OUT:
                        if (waitforDrawers(blueboi1, blueboi2)) {
                            bigflip1.setPosition(.8);
                            bigflip2.setPosition(0.48);
                            littleflip.setPosition(0.5);

                            drawerTimer.reset();
                            drawerState = State.DRAWER_FLIP_IN;
                        }
                        break;

                    case DRAWER_FLIP_IN:
                        if (drawerTimer.seconds() >= 1) {
                            littleflip.setPosition(0.7);
                            bigflip1.setPosition(0.6);
                            bigflip2.setPosition(1);

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
                        if ((drawerTimer.seconds() >= 3) && waitforDrawers(blueboi1, blueboi2)) {
                            littleflip.setPosition(0.5);
                            untoPosition(blueboi1);
                            untoPosition(blueboi2);

                            drawerState = State.DRAWER_START;
                            timer.reset();
                        }
                        break;
                    default:
                        drawerState = State.DRAWER_START;
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