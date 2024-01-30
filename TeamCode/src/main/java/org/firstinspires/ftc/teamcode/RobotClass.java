/*
    TODO: Add movement methods
    TODO: Add camera
    TODO: Add sensors (gyro, distance)
 */

package org.firstinspires.ftc.teamcode;

/*
    Imports
 */

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.VisionPortal;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class RobotClass {

    //initializing variables
    private LinearOpMode myOpMode = null;

    //initializing motor variables
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    
    public BNO055IMU imu;
    public Orientation angles;
    
    public Servo intakeDoor;
    public DistanceSensor leftDistanceSensor;
    public DistanceSensor rightDistanceSensor;
    
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public AprilTagProcessor aprilTag;
    public VisionPortal visionPortal;
    
    // Creating Enum for position of team prop
    public enum Position {
        LEFT,
        RIGHT,
        CENTER
    }

    public RobotClass(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init(HardwareMap hardwareMap) {
        //initializing motors
        initMotors(hardwareMap);

        //initializing IMU
        try {
            initGyro(hardwareMap);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        
        //initializing sensors
        initSensors(hardwareMap);
    
        
        
        //initializing servos
//        intakeDoor = hardwareMap.get(Servo.class, "intakeDoor");
    }

    //initalizing motors
    private void initMotors(HardwareMap hardwareMap) {
        //assigning motor variables to configuration name
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //setting zero power behavior to brake
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //initializing IMU
    private void initGyro(HardwareMap hardwareMap) throws InterruptedException{
        //TODO: Test different IMU configs

        // Set up the parameters with which we will use our IMU.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample OpMode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        //hardware mapping and initializing imu
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        while(!imu.isGyroCalibrated()){
            sleep(10);
        }
        dashboardTelemetry.addData("Status", imu.isGyroCalibrated());
        dashboardTelemetry.update();
    }
    
    private void initSensors(HardwareMap hardwareMap){
        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "left");
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "right");
    }

    //initializing camera
    private void initCamera(HardwareMap hardwareMap) {
        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
    }

    //resetting encoders
    public void resetEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //stops all motors
    public void stopMotors() {
        //setting mode of motors
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //setting power to 0
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void moveWithoutEncoders(double powerLeft, double powerRight, int timeInMs) throws InterruptedException {
        //setting mode of motors
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //setting power of motors
        frontLeft.setPower(powerLeft);
        frontRight.setPower(powerRight);
        backLeft.setPower(powerLeft);
        backRight.setPower(powerRight);
        //waiting while running motors
        sleep(timeInMs);
        //stops motors
        stopMotors();
    }


    //Moving using encoders
    public void moveStraightWithEncoders(double power, double cm) throws InterruptedException {
        //setting number of ticks per 10 cm to get number of ticks per cm
        int ticksPer40cm = 1000;
        int ticksPerCm = ticksPer40cm / 40;
        int target = (int) Math.round(cm * ticksPerCm);

        //resetting
        resetEncoders();

        //setting target position of motors
        frontLeft.setTargetPosition(target);
        frontRight.setTargetPosition(target);
        backLeft.setTargetPosition(target);
        backRight.setTargetPosition(target);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        //waiting while the motors are busy
        while (frontLeft.isBusy()){
            sleep(50);
        }

        //stopping all motors
        stopMotors();

        resetEncoders();
    }

    //turning with gyro code
    double angleMinThreshold = 0.5;
    double angleMaxThreshold = 10;
    double minCorrectionPower = 0.2;
    double maxCorrectionPower = 2.0;

    public void gyroTurning(double targetAngleDegrees) throws InterruptedException {
        boolean run = true;
        while (run) {
            angles = imu.getAngularOrientation();
            dashboardTelemetry.addData("angle", angles.firstAngle);
            dashboardTelemetry.update();
            //using gyro
            if (angles.firstAngle >= targetAngleDegrees - angleMinThreshold && angles.firstAngle <= targetAngleDegrees + angleMinThreshold) {
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                sleep(500);
                if (angles.firstAngle >= targetAngleDegrees - angleMinThreshold && angles.firstAngle <= targetAngleDegrees + angleMinThreshold) {
                    run = false;
                    return;
                }
            } else if (angles.firstAngle >= targetAngleDegrees) {
                if (angles.firstAngle <= targetAngleDegrees + angleMaxThreshold) {
                    frontLeft.setPower(minCorrectionPower);
                    frontRight.setPower(-minCorrectionPower);
                    backLeft.setPower(minCorrectionPower);
                    backRight.setPower(-minCorrectionPower);
                } else {
                    frontLeft.setPower(maxCorrectionPower);
                    frontRight.setPower(-maxCorrectionPower);
                    backLeft.setPower(maxCorrectionPower);
                    backRight.setPower(-maxCorrectionPower);
                }
            } else if (angles.firstAngle <= targetAngleDegrees) {
                if (angles.firstAngle >= targetAngleDegrees - angleMaxThreshold) {
                    frontLeft.setPower(-minCorrectionPower);
                    frontRight.setPower(minCorrectionPower);
                    backLeft.setPower(-minCorrectionPower);
                    backRight.setPower(minCorrectionPower);

                } else {
                    frontLeft.setPower(-maxCorrectionPower);
                    frontRight.setPower(maxCorrectionPower);
                    backLeft.setPower(-maxCorrectionPower);
                    backRight.setPower(maxCorrectionPower);
                }
            }
            stopMotors();
        }
    }

    //strafing class with power and direction as parameters
    public enum Direction {
        LEFT, RIGHT
    }
    public void strafing(Direction direction, double power, int timeInMs) throws InterruptedException {
        if (direction == Direction.LEFT) {
            frontLeft.setPower(-power);
            backLeft.setPower(power);
            frontRight.setPower(-power);
            backRight.setPower(power);
        } else if (direction == Direction.RIGHT) {
            frontLeft.setPower(power);
            backLeft.setPower(-power);
            frontRight.setPower(power);
            backRight.setPower(-power);
        } else {
            dashboardTelemetry.addData("Error", "Invalid direction");
            dashboardTelemetry.update();
        }
        sleep(timeInMs);
        stopMotors();
    }

    //Using AprilTag to find the team prop
    public Position findTeamProp() {
        double distanceLeft = leftDistanceSensor.getDistance(DistanceUnit.CM);
        double distanceRight = rightDistanceSensor.getDistance(DistanceUnit.CM);
        
        dashboardTelemetry.addData("Distance to the left", distanceLeft);
        dashboardTelemetry.addData("Distance to the right", distanceRight);
        dashboardTelemetry.update();
        
        if (distanceLeft < 13) {
            return Position.LEFT;
        } else if (distanceRight < 13) {
            return Position.RIGHT;
        } else {
            return Position.CENTER;
        }
    }
}
