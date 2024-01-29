package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.lang.Math;

@TeleOp
public class ArcadeDrive extends LinearOpMode {

    //Initializing motor variables
    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;
    
    Servo deployer;
    DcMotor winch;
    
    Servo launcher;
    
    DistanceSensor leftDistanceSensor;
    DistanceSensor rightDistanceSensor;

    public void runOpMode(){
        //Assigning configuration name to variable (for frontLeft, backLeft, frontRight, backRight)
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        deployer = hardwareMap.get(Servo.class, "deployer");
        winch = hardwareMap.get(DcMotor.class, "winch");
        
        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "left");
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "right");
        
        launcher = hardwareMap.get(Servo.class, "launcher");

        //setting direction of motors
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        
        //setting initial position of the deployer servo
        deployer.setPosition(0.55);
        
        launcher.setPosition(0);
        
        //Initializing FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
    
        dashboardTelemetry.addLine("Robot Initialized");
        dashboardTelemetry.update();
        
        //Waiting for Start button to be pressed
        waitForStart();
        
        dashboardTelemetry.clear();

        //Looping while the opmode is running
        double throttle = 0;
        double turn = 0;
        boolean strafeR = false;
        boolean strafeL = false;
        while (opModeIsActive()){
            //defining driving variables (throttle = moving)
            throttle = -gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;
            strafeR = gamepad1.right_bumper;
            strafeL = gamepad1.left_bumper;


            //setting power for forward-backward movement
            frontLeft.setPower(throttle);
            backLeft.setPower(throttle);
            frontRight.setPower(throttle);
            backRight.setPower(throttle);

            //setting up strafing
            if(strafeR) {
                frontLeft.setPower(0.75);
                backLeft.setPower(-0.75);
                frontRight.setPower(0.75);
                backRight.setPower(-0.75);
            } else if (strafeL) {
                frontLeft.setPower(-0.75);
                backLeft.setPower(0.75);
                frontRight.setPower(-0.75);
                backRight.setPower(0.75);
            }

            //setting power for turning
            frontLeft.setPower(turn);
            backLeft.setPower(turn);
            frontRight.setPower(-turn);
            backRight.setPower(-turn);

            //raising of the hanging mechanism
            if (gamepad2.a) {
                winch.setPower(1);
            } else if (gamepad2.b) {
                winch.setPower(-1);
            } else {
                winch.setPower(0);
            }
            
            if (gamepad2.dpad_up) {
                deployer.setPosition(0.17);
            }
            
            launcher.setPosition(gamepad2.left_stick_y);
            
//            intakeDoor.setPosition(gamepad2.left_stick_y);
            
            dashboardTelemetry.addData("deployer position", deployer.getPosition());
            dashboardTelemetry.addData("left distance sensor", leftDistanceSensor.getDistance(DistanceUnit.CM));
            dashboardTelemetry.addData("right distance sensor", rightDistanceSensor.getDistance(DistanceUnit.CM));
            dashboardTelemetry.update();
        }
    }
}
