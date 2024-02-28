package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.lang.Math;

@TeleOp
public class ArcadeDrive extends LinearOpMode {
    Servo deployer;
    DcMotor winch;
    
    Servo launcher;
    
    boolean isDroneLaunched = false;
    
    // Setting variable for enabling endgame functions
    boolean endgame = false;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
  
    public void runOpMode(){
        //Assigning configuration name to variable (for frontLeft, backLeft, frontRight, backRight)
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        deployer = hardwareMap.get(Servo.class, "deployer");
        winch = hardwareMap.get(DcMotor.class, "winch");
        
        launcher = hardwareMap.get(Servo.class, "launcher");
        
        winch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Creating new elapsed time object for timer
        ElapsedTime timer = new ElapsedTime();
        
        //setting initial position of servos
        deployer.setPosition(0.55);
        
        launcher.setPosition(0);
        
        drive.setPoseEstimate(PoseStorage.currentPose);

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
        while (!isStopRequested()){
            
            if (gamepad1.left_bumper) {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                1,
                                -gamepad1.right_stick_x
                        )
                );
            } else if (gamepad1.right_bumper) {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                -1,
                                -gamepad1.right_stick_x
                        )
                );
            } else {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                              0,
                                -gamepad1.right_stick_x
                        )
                );
            }
            
            drive.update();
            
            if (timer.time() > 90 && !endgame) {
                gamepad1.rumble(0.75, 0.75, 1500);
                endgame = true;
            }
            
            if (endgame) {
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
                
                launcher.setPosition(gamepad2.left_trigger);
                
                if (gamepad1.a) {
                    launcher.setPosition(1);
                    isDroneLaunched = true;
                }
            }
            
            if (gamepad1.back){
                endgame = true;
            }

            dashboardTelemetry.addData("time", timer.time());
            dashboardTelemetry.update();
            telemetry.addData("deployer position", deployer.getPosition());
            telemetry.update();
        }
    }
}
