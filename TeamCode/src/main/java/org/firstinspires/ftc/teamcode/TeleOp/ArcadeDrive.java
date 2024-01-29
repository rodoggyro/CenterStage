package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    
    // Setting variable for enabling endgame functions
    boolean endgame = false;

    public void runOpMode(){
        //Assigning configuration name to variable (for frontLeft, backLeft, frontRight, backRight)
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        deployer = hardwareMap.get(Servo.class, "deployer");
        winch = hardwareMap.get(DcMotor.class, "winch");
        
        launcher = hardwareMap.get(Servo.class, "launcher");

        //setting direction of motors
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Creating new elapsed time object for timer
        ElapsedTime timer = new ElapsedTime();
        
        //setting initial position of servos
        deployer.setPosition(0.55);
        
        launcher.setPosition(0);
        
        //Waiting for Start button to be pressed
        waitForStart();
        
        //starting timer
        timer.reset();

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
                frontLeft.setPower(-0.75);
                backLeft.setPower(0.75);
                frontRight.setPower(0.75);
                backRight.setPower(-0.75);
            } else if (strafeL) {
                frontLeft.setPower(0.75);
                backLeft.setPower(-0.75);
                frontRight.setPower(-0.75);
                backRight.setPower(0.75);
            }

            //setting power for turning
            frontLeft.setPower(-turn);
            backLeft.setPower(-turn);
            frontRight.setPower(turn);
            backRight.setPower(turn);
            
            if (timer.time() > 120 && !endgame) {
                gamepad1.rumble(0.75, 0.75, 1500);
                endgame = true;
            }

            if (endgame) {
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
    
                if (gamepad2.left_bumper) {
                    launcher.setPosition(1);
                }
            }
            
            
            
            telemetry.addData("deployer position", deployer.getPosition());
            telemetry.update();
        }
    }
}
