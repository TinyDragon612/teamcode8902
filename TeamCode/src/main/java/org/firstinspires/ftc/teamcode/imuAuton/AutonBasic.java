package org.firstinspires.ftc.teamcode.imuAuton;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Disabled
@Autonomous(name="AutonBasic", group="Pushbot")
public class AutonBasic extends LinearOpMode{

    private DcMotor.ZeroPowerBehavior brake = DcMotor.ZeroPowerBehavior.BRAKE;
    private DcMotor.ZeroPowerBehavior floatt = DcMotor.ZeroPowerBehavior.FLOAT;

    /* Declare OpMode members. */
    HardwarePushbot         robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();


    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;

    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;

    private int errorBound = 60;

    int macro = 0;

    int ext = 900;

    private DcMotorEx alex; //hanger

    private DcMotorEx blueboi1, blueboi2; //drawers

    private DcMotorEx spinyboi; //intake

    private Servo bigflip1, bigflip2; //bring pixel holder out

    private Servo floppy; //release or hold hanger

    private Servo littleflip; // release or hold pixel in holder

    private Servo lettuce; //release or hold intake

    private Servo launcher; // launches paper airplane

    private PIDController pidDrive;

    //  ElapsedTime Bill =  new ElapsedTime(bart);

    int level = 0;

    @Override
    public void runOpMode() throws InterruptedException{

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

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


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if(opModeIsActive()) {
            resetwheelpos();
            robot.forward(1000);
            //driveStraight(1000);
            waitforwheels();
            robot.forward(-1000);
            //driveStraight(-1000);
            waitforwheels();
            final int STAGE = 1;
            if (STAGE == 1) {
                switchywichy();
                sleep(1000);
                lobertRockatel(90, 3);
                turnTo(-90);
                sleep(3000);
                lobertRockatel(180, 3);
                turnTo(-180);
                sleep(300);

                /*
                movehorizontally(200, 0.6);
                waitforwheels();
                resetwheelpos();
                movevertically(-200, 0.6);
                waitforwheels();
                lobertRockatel(17, 3);
                turnTo(-17);
                 */

                while(true) {
                    telemetry.addData("angle" , getAngle());
                    telemetry.update();
                }

            } else if (STAGE == 2) {
                turnPID(90);
            }

        }

    }
    // resets currAngle Value
    public void resetAngle() {
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currAngle = 0;
    }

    public double getAngle() {

        // Get current orientation
        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // Change in angle = current angle - previous angle
        double deltaAngle = orientation.firstAngle - lastAngles.firstAngle;

        // Gyro only ranges from -179 to 180
        // If it turns -1 degree over from -179 to 180, subtract 360 from the 359 to get -1
        if (deltaAngle < -180) {
            deltaAngle += 360;
        } else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }

        // Add change in angle to current angle to get current angle
        currAngle += deltaAngle;
        lastAngles = orientation;
        telemetry.addData("gyro", orientation.firstAngle);
        return currAngle;
    }

    public void turn(double degrees){
        resetAngle();

        double error = degrees;

        while (opModeIsActive() && Math.abs(error) > 5) {
            double motorPower = (error < 0 ? -0.3 : 0.3);
            robot.setMotorPower(-motorPower, motorPower, -motorPower, motorPower);
            error = degrees - getAngle();
            telemetry.addData("error", error);
            telemetry.update();
        }

        robot.setAllPower(0);
    }

    public void turnTo(double degrees){

        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        System.out.println(orientation.firstAngle);
        double error = degrees - orientation.firstAngle;

        if (error > 180) {
            error -= 360;
        } else if (error < -180) {
            error += 360;
        }

        turn(error);
    }

    public double getAbsoluteAngle() {
        return robot.imu.getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES
        ).firstAngle;
    }

    public void turnPID(double degrees) {
        turnToPID(degrees + getAbsoluteAngle());
    }

    void turnToPID(double targetAngle) {
        TurnPIDController pid = new TurnPIDController(targetAngle, 0.01, 0, 0.003);
        telemetry.setMsTransmissionInterval(50);
        // Checking lastSlope to make sure that it's not oscillating when it quits
        while (Math.abs(targetAngle - getAbsoluteAngle()) > 0.5 || pid.getLastSlope() > 0.75) {
            double motorPower = pid.update(getAbsoluteAngle());
            robot.setMotorPower(-motorPower, motorPower, -motorPower, motorPower);

            telemetry.addData("Current Angle", getAbsoluteAngle());
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Slope", pid.getLastSlope());
            telemetry.addData("Power", motorPower);
            telemetry.update();
        }
        robot.setAllPower(0);
    }

    public void lobertRockatel(double turnAngle, double timeoutS) {

        //    sleep(500);
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double speed= .5;
        double oldDegreesLeft=turnAngle;
        double scaledSpeed=speed;
        double targetHeading=angles.firstAngle+turnAngle;
        double oldAngle=angles.firstAngle;

        if(targetHeading<-180) {targetHeading+=360;}
        if(targetHeading>180){targetHeading-=360;}

        double degreesLeft = ((int)(Math.signum(angles.firstAngle-targetHeading)+1)/2)*(360-Math.abs(angles.firstAngle-targetHeading))+(int)(Math.signum(targetHeading-angles.firstAngle)+1)/2*Math.abs(angles.firstAngle-targetHeading);
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < timeoutS && degreesLeft>1 && oldDegreesLeft-degreesLeft>=0) {
            //check to see if we overshot target
            scaledSpeed=degreesLeft/(100+degreesLeft)*speed;
            if(scaledSpeed>1){scaledSpeed=.1;}
            robot.bottomLeft.setPower(scaledSpeed*1.3); //extra power to back wheels
            robot.bottomRight.setPower(-1*scaledSpeed*1.3); //due to extra weight
            robot.topLeft.setPower(scaledSpeed);
            robot.bottomRight.setPower(-1*scaledSpeed);
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            oldDegreesLeft=degreesLeft;
            degreesLeft = ((int)(Math.signum(angles.firstAngle-targetHeading)+1)/2)*(360-Math.abs(angles.firstAngle-targetHeading))+(int)(Math.signum(targetHeading-angles.firstAngle)+1)/2*Math.abs(angles.firstAngle-targetHeading);
            if(Math.abs(angles.firstAngle-oldAngle)<1){speed*=1.1;}
            //bump up speed to wheels in case robot stalls before reaching target
            oldAngle=angles.firstAngle;
        }
        robot.setAllPower(0.0);
        //  sleep(250); //small pause at end of turn
    }

    public void driveStraight(double power){
        pidDrive = new PIDController(.05, 0, 0);

        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, power);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();

        double correction = pidDrive.performPID(getAngle());

        // set power levels.

        robot.topLeft.setPower(power - correction);
        robot.bottomLeft.setPower(power - correction);
        robot.topRight.setPower(power + correction);
        robot.bottomRight.setPower(power + correction);

    }


    //END OF IMU CODE

    public void waitforDrawer(DcMotor george) {
        while(!(george.getCurrentPosition() > george.getTargetPosition() - errorBound && george.getCurrentPosition() < george.getTargetPosition() + errorBound));
    }

    public void waitforDrawers(DcMotor george, DcMotor BobbyLocks) {
        while(!((george.getCurrentPosition() > george.getTargetPosition() - errorBound && george.getCurrentPosition() < george.getTargetPosition() + errorBound) &&
                (BobbyLocks.getCurrentPosition() > BobbyLocks.getTargetPosition() - errorBound && BobbyLocks.getCurrentPosition() < BobbyLocks.getTargetPosition() + errorBound)));
    }

    public void nostall(DcMotor Harry) {
        Harry.setZeroPowerBehavior(floatt);
        Harry.setPower(0);
    }

    public void stall(DcMotor DcMotar) {
        DcMotar.setZeroPowerBehavior(brake);
        DcMotar.setPower(0);
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

    public void resetwheelpos() {
        stoop();
        robot.topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.topRight.setTargetPosition(0);
        robot.topRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.topRight.setPower(0);
        robot.bottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bottomRight.setTargetPosition(0);
        robot.bottomRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bottomRight.setPower(0);
        robot.topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.topLeft.setTargetPosition(0);
        robot.topLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.topLeft.setPower(0);
        robot.bottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bottomLeft.setTargetPosition(0);
        robot.bottomLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bottomLeft.setPower(0);
    }

    public void waitforwheels() {
        while (robot.topLeft.getCurrentPosition() != robot.topLeft.getTargetPosition() && robot.topRight.getCurrentPosition() != robot.topRight.getTargetPosition() && robot.bottomLeft.getCurrentPosition() != robot.bottomLeft.getTargetPosition() && robot.bottomRight.getCurrentPosition() != robot.bottomRight.getTargetPosition())
            ;
        resetwheelpos();
    }

    public void movevertically(int position, double power) {
        robot.topRight.setTargetPosition(robot.topRight.getCurrentPosition() + position);
        robot.bottomRight.setTargetPosition(robot.bottomRight.getCurrentPosition() + position);
        robot.topLeft.setTargetPosition(robot.topLeft.getCurrentPosition() + position);
        robot.bottomLeft.setTargetPosition(robot.bottomLeft.getCurrentPosition() + position);
        robot.topRight.setPower(power);
        robot.topLeft.setPower(power);
        robot.bottomRight.setPower(power);
        robot.bottomLeft.setPower(power);
    }

    public void stoop() {
        robot.topRight.setPower(0);
        robot.topLeft.setPower(0);
        robot.bottomRight.setPower(0);
        robot.bottomLeft.setPower(0);
    }

    public void movehorizontally(int position, double power) {
        robot.topRight.setTargetPosition(robot.topRight.getCurrentPosition() + position);
        robot.bottomRight.setTargetPosition(-(robot.topRight.getCurrentPosition() + position));
        robot.topLeft.setTargetPosition(-(robot.topRight.getCurrentPosition() + position));
        robot.bottomLeft.setTargetPosition(robot.topRight.getCurrentPosition() + position);
        robot.topRight.setPower(-power);
        robot.bottomRight.setPower(power);
        robot.topLeft.setPower(power);
        robot.bottomLeft.setPower(-power);
    }

    public void switchywichy() {
        robot.topLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.bottomLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.topRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.bottomRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}