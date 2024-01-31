package org.firstinspires.ftc.teamcode;

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

@Disabled
@TeleOp(name="BabyMode", group="Linear Opmode")
public class BabyMode extends LinearOpMode {

    private DcMotorEx topRight, bottomRight, topLeft, bottomLeft; //wheels

    @Override
    public void runOpMode() {

        topRight = hardwareMap.get(DcMotorEx.class, "topRight");
        topLeft = hardwareMap.get(DcMotorEx.class, "topLeft");
        bottomRight = hardwareMap.get(DcMotorEx.class, "bottomRight");
        bottomLeft = hardwareMap.get(DcMotorEx.class, "bottomLeft");

        topRight.setDirection(DcMotorEx.Direction.FORWARD);
        bottomRight.setDirection(DcMotorEx.Direction.REVERSE);
        topLeft.setDirection(DcMotorEx.Direction.REVERSE);
        bottomLeft.setDirection(DcMotorEx.Direction.REVERSE);

        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                telemetry.addData("Status", "Running");
                telemetry.addData("Program", "Android Studio");

                topRight.setPower((((gamepad1.left_stick_y + gamepad1.left_stick_x)) + (gamepad1.right_stick_x)) * .25);
                topLeft.setPower((((-gamepad1.left_stick_y + gamepad1.left_stick_x)) + ((gamepad1.right_stick_x))) * .25);
                bottomRight.setPower((((gamepad1.left_stick_y + -gamepad1.left_stick_x)) + (gamepad1.right_stick_x)) * .25);
                bottomLeft.setPower((((-gamepad1.left_stick_y + -gamepad1.left_stick_x)) + (gamepad1.right_stick_x)) * .25);

            }
        }

    }
}