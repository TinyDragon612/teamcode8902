package org.firstinspires.ftc.teamcode.IMUAuton;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Disabled
/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwarePushbot
{
    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    /* Public OpMode members. */
    public DcMotor  topLeft;
    public DcMotor  topRight;
    public DcMotor  bottomLeft;
    public DcMotor  bottomRight;

    public BNO055IMU imu;

    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        topLeft   = hwMap.get(DcMotor.class, "topLeft");
        topRight  = hwMap.get(DcMotor.class, "topRight");
        bottomLeft   = hwMap.get(DcMotor.class, "bottomLeft");
        bottomRight  = hwMap.get(DcMotor.class, "bottomRight");
        topRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomRight.setDirection(DcMotorSimple.Direction.FORWARD);
        topLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        topLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all motors to zero power
        setAllPower(0);
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topLeft.setTargetPosition(0);
        topLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        topLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        bottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomLeft.setTargetPosition(0);
        bottomLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRight.setTargetPosition(0);
        topRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        topRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        bottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomRight.setTargetPosition(0);
        bottomRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
    //Set power to all motors
    public void setAllPower(double p){
        setMotorPower(p,p,p,p);
    }

    public void setMotorPower(double lF, double rF, double lB, double rB){
        topLeft.setPower(lF);
        bottomLeft.setPower(lB);
        bottomRight.setPower(rB);
        topRight.setPower(rF);
    }

    public void resetwheelpos() {
        stoop();
        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRight.setTargetPosition(0);
        topRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        topRight.setPower(0);
        bottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomRight.setTargetPosition(0);
        bottomRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomRight.setPower(0);
        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topLeft.setTargetPosition(0);
        topLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        topLeft.setPower(0);
        bottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomLeft.setTargetPosition(0);
        bottomLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomLeft.setPower(0);
    }

    public void waitforwheels() {
        while (topLeft.getCurrentPosition() != topLeft.getTargetPosition() && topRight.getCurrentPosition() != topRight.getTargetPosition() && bottomLeft.getCurrentPosition() != bottomLeft.getTargetPosition() && bottomRight.getCurrentPosition() != bottomRight.getTargetPosition())
            ;
        resetwheelpos();
    }

    public void forward(int distance){
        moveVertically(bottomLeft, distance, 0.5);
        moveVertically(bottomRight, distance, 0.5);
        moveVertically(topRight, distance, 0.5);
        moveVertically(topLeft, distance, 0.5);
    }

    public void moveVertically(DcMotor mot, int position, double power){
        mot.setPower(0);
        mot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mot.setTargetPosition(0);
        mot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mot.setPower(0);

        mot.setTargetPosition(position);
        mot.setPower(power);
    }

    public void runtoPosition(DcMotor John) {
        John.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        John.setTargetPosition(0);
        John.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        John.setPower(0);
    }
    public void untoPosition(DcMotor Neil) {
        Neil.setPower(0);
        Neil.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
    }

    public void stoop() {
        topRight.setPower(0);
        topLeft.setPower(0);
        bottomRight.setPower(0);
        bottomLeft.setPower(0);
    }

    public void movehorizontally(int position, double power) {
        topRight.setTargetPosition(topRight.getCurrentPosition() + position);
        bottomRight.setTargetPosition(-(topRight.getCurrentPosition() + position));
        topLeft.setTargetPosition(-(topRight.getCurrentPosition() + position));
        bottomLeft.setTargetPosition(topRight.getCurrentPosition() + position);
        topRight.setPower(-power);
        bottomRight.setPower(power);
        topLeft.setPower(power);
        bottomLeft.setPower(-power);
    }


}