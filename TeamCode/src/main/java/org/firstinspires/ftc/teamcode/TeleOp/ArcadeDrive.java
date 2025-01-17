package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


import org.firstinspires.ftc.robotcore.external.Telemetry;

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
    
    Servo claw;
    Servo clawRotator;
    
    boolean isDroneLaunched = false;
    
    // Setting variable for enabling endgame functions
    boolean endgame = false;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
  
    public void runOpMode(){
        //Assigning configuration name to variable (for frontLeft, backLeft, frontRight, backRight)
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        
        deployer = hardwareMap.get(Servo.class, "deployer");
        winch = hardwareMap.get(DcMotor.class, "winch");
        
        launcher = hardwareMap.get(Servo.class, "launcher");
        
        claw = hardwareMap.get(Servo.class, "claw");
        claw.setPosition(1);
        clawRotator = hardwareMap.get(Servo.class, "clawRotator");
        clawRotator.setPosition(0.25);
        
        //setting direction of motors
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        
        winch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Creating new elapsed time object for timer
        ElapsedTime timer = new ElapsedTime();
        
        //setting initial position of servos
        deployer.setPosition(0.55);
        
        launcher.setPosition(0);
        

        //Waiting for Start button to be pressed
        waitForStart();
        
        //starting timer
        timer.reset();
        //Initializing FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
    
        dashboardTelemetry.addLine("Robot Initialized");
        dashboardTelemetry.update();
        
        //Looping while the opmode is running
        double throttle = 0;
        double turn = 0;
        double strafing = 0;
        while (opModeIsActive()){
            //defining driving variables (throttle = moving)
            throttle = -gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;
            strafing = gamepad1.left_stick_x;

            //setting power for forward-backward movement
            frontLeft.setPower(throttle);
            backLeft.setPower(throttle);
            frontRight.setPower(throttle);
            backRight.setPower(throttle);
            
            //setting up strafing
            if (gamepad1.left_bumper) {
                frontLeft.setPower(0.75);
                backLeft.setPower(-0.75);
                frontRight.setPower(-0.75);
                backRight.setPower(0.75);
            }else if (gamepad1.right_bumper) {
                frontLeft.setPower(-0.75);
                backLeft.setPower(0.75);
                frontRight.setPower(0.75);
                backRight.setPower(-0.75);
            }
          
            //setting power for turning
            frontLeft.setPower(turn);
            backLeft.setPower(turn);
            frontRight.setPower(-turn);
            backRight.setPower(-turn);
            
            if (timer.time() > 90 && !endgame) {
                gamepad1.rumble(0.75, 0.75, 1500);
                endgame = true;
            }
            
            if (endgame) {
                clawRotator.setPosition(0.5);
                //raising of the hanging mechanism
                if (gamepad1.left_trigger > 0) {
                    winch.setPower(-gamepad1.left_trigger);
                } else if (gamepad1.right_trigger > 0) {
                    winch.setPower(gamepad1.right_trigger);
                } else {
                    winch.setPower(0);
                }
                
                if (gamepad1.b && isDroneLaunched) {
                    deployer.setPosition(0.17);
                }
                
                if (gamepad1.a) {
                    launcher.setPosition(1);
                    isDroneLaunched = true;
                }
            }
            
            if (gamepad1.back){
                endgame = true;
            }
            
            if (gamepad1.x) {
                clawRotator.setPosition(0.65);
                sleep(1500);
                claw.setPosition(0);
                sleep(500);
                clawRotator.setPosition(0.25);
            }

            dashboardTelemetry.addData("time", timer.time());
            dashboardTelemetry.update();
            telemetry.addData("deployer position", deployer.getPosition());
            telemetry.addData("claw", claw.getPosition());
            telemetry.addData("claw rotator", clawRotator.getPosition());
            telemetry.update();
        }
    }
}
