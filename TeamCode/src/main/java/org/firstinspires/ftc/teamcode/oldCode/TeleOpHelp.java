package org.firstinspires.ftc.teamcode.oldCode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@Disabled
@TeleOp(name="TeleOpBad", group="Linear Opmode")
//@Disabled
public class TeleOpHelp extends LinearOpMode {

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

    private int errorBound = 60;

    boolean weGoin;
    int height;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
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


        telemetry.update();

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        topRight.setDirection(DcMotorEx.Direction.FORWARD);
        bottomRight.setDirection(DcMotorEx.Direction.FORWARD);
        topLeft.setDirection(DcMotorEx.Direction.REVERSE);
        bottomLeft.setDirection(DcMotorEx.Direction.REVERSE);

        blueboi1.setDirection(DcMotorEx.Direction.REVERSE);
        blueboi2.setDirection(DcMotorEx.Direction.FORWARD);
        alex.setDirection(DcMotorEx.Direction.REVERSE);
        spinyboi.setDirection(DcMotorEx.Direction.REVERSE);

        //bigflip1.setPosition(0.6);
        //bigflip2.setPosition(1);

        lettuce.setPosition(0);
        //floppy.setPosition(1);
        littleflip.setPosition(0.7);
        launcher.setPosition(0.5);

        // Wait for the game to start (driver presses PLAY)
        telemetry.update();
        waitForStart();

        if (opModeIsActive()){
            blueboi1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            blueboi2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            alex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            blueboi1.setTargetPosition(0);
            blueboi2.setTargetPosition(0);
            alex.setTargetPosition(0);

            blueboi1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            blueboi2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            alex.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            while (opModeIsActive()) {

                telemetry.addData("Status", "Running");
                telemetry.addData("Program", "Android Studio");
                telemetry.addData("bigflip1:", bigflip1.getPosition());
                telemetry.addData("bigflip2:", bigflip2.getPosition());
                telemetry.addData("Blueboi1: ", blueboi1.getCurrentPosition());
                telemetry.addData("Blueboi2: ", blueboi2.getCurrentPosition());
                telemetry.update();


                //GAMEPAD1 CONTROLS

                topRight.setPower(((gamepad1.left_stick_y + gamepad1.left_stick_x)) + (gamepad1.right_stick_x));
                topLeft.setPower(((-gamepad1.left_stick_y + gamepad1.left_stick_x)) + ((gamepad1.right_stick_x)));
                bottomRight.setPower(((gamepad1.left_stick_y + -gamepad1.left_stick_x)) + (gamepad1.right_stick_x));
                bottomLeft.setPower(((-gamepad1.left_stick_y + -gamepad1.left_stick_x)) + (gamepad1.right_stick_x));

                if(gamepad1.y){
                    setDrawerHeight(900);
                }

                if(gamepad1.b){
                    setDrawerHeight(750);
                }

                if(gamepad1.a){
                    setDrawerHeight(500);
                }

                if(gamepad1.x){
                    bringDrawersDown();
                }

                if(gamepad1.dpad_down){
                    littleflip.setPosition(0.7);
                }

                if(gamepad1.dpad_up){
                    littleflip.setPosition(0.5);
                }

                if(gamepad1.right_bumper){
                    launcher.setPosition(0);
                }

                if(gamepad1.left_bumper){
                    lettuce.setPosition(0.55);
                }

                //GAMEPAD2 CONTROLS
                spinyboi.setPower(gamepad2.left_stick_y); //send power to motor to spin intake
                alex.setPower(gamepad2.right_stick_y); //send power to motor to lift/hang bot

                if(gamepad2.right_bumper){
                    floppy.setPosition(0.55); //release the hanger
                }

                if(gamepad2.left_bumper){
                    lettuce.setPosition(0.55); //lower the intake
                }

                if(gamepad2.y){
                    setDrawerHeight(900);
                }

                if(gamepad2.b){
                    setDrawerHeight(750);
                }

                if(gamepad2.a){
                    setDrawerHeight(500);
                }

                if(gamepad2.x){
                    bringDrawersDown();
                }

                if(gamepad2.dpad_down){
                    littleflip.setPosition(0.7);
                }

                if(gamepad2.dpad_up){
                    littleflip.setPosition(0.5);
                }


                //TESTING
                if(gamepad2.dpad_right){
                    //bigflip1.setPosition(1);
                    //bigflip2.setPosition(1);

                    bigflip1.setPosition(0.6);
                    bigflip2.setPosition(1);
                }

                if(gamepad2.dpad_left){
                    //bigflip1.setPosition(0);
                    //bigflip2.setPosition(0);

                    bigflip1.setPosition(.8);
                    bigflip2.setPosition(0.48);
                }




            }


        }

    }

    public void bringDrawersDown(){
        littleflip.setPosition(0.7);
        bigflip1.setPosition(0.6);
        bigflip2.setPosition(1);

        sleep(500);

        movevertically(blueboi1, -height-50, 1);
        movevertically(blueboi2, -height-50, 1);
        waitforDrawers(blueboi1, blueboi2);
        sleep(300);
        littleflip.setPosition(0.5);
        runtoPosition(blueboi1);
        runtoPosition(blueboi2);

        /*
        while(!(blueboi1.getCurrentPosition() < 5 && blueboi2.getCurrentPosition() < 5)){
            runtoPosition(blueboi1);
            runtoPosition(blueboi2);

            if(gamepad1.dpad_right){
                break;
            }
        }

        OR

        waitforDrawers(blueboi1, blueboi2);
        if (getTime() > 300000000){
            sleep(300);
            littleflip.setPosition(0.5);

            runtoPosition(blueboi1);
            runtoPosition(blueboi2);
        }
         */
    }

    public void setDrawerHeight(int h){
        height = h;
        movevertically(blueboi1, h, 1);
        movevertically(blueboi2, h, 1);

        waitforDrawers(blueboi1, blueboi2);

        bigflip1.setPosition(.8);
        bigflip2.setPosition(0.48);
    }

    /*
    public void moveTogether(DcMotorEx lenny, DcMotorEx fred, int position, double power){
        untoPosition(lenny);
        untoPosition(fred);

        runtoPosition(lenny);

        lenny.setTargetPosition(lenny.getCurrentPosition() + position);
        fred.setTargetPosition(lenny.getCurrentPosition() + position);

        lenny.setPower(power);
        fred.setPower(power);
    }
     */

    public void waitforDrawer(DcMotor george) {
        while(!(george.getCurrentPosition() > george.getTargetPosition() - errorBound && george.getCurrentPosition() < george.getTargetPosition() + errorBound));
    }

    public void waitforDrawers(DcMotor george, DcMotor BobbyLocks) {
        while(!((george.getCurrentPosition() > george.getTargetPosition() - errorBound && george.getCurrentPosition() < george.getTargetPosition() + errorBound) &&
                (BobbyLocks.getCurrentPosition() > BobbyLocks.getTargetPosition() - errorBound && BobbyLocks.getCurrentPosition() < BobbyLocks.getTargetPosition() + errorBound)));
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
    public long getTime(){
        long start = System.nanoTime();
        long finish = System.nanoTime();
        long timeElapsed = finish - start;

        return timeElapsed;
    }

}