package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotClass;

@Autonomous
public class RedBackstage extends LinearOpMode {
    //Instantiate robot class
    RobotClass teamBot = new RobotClass(this);
    
    int parkingPosition  = 1;
    
    public void runOpMode() throws InterruptedException {
        //initialize robot
        teamBot.init(hardwareMap);

        waitForStart();
        
        teamBot.moveStraightWithEncoders(0.6, 72);
//        RobotClass.Position posOfTag = teamBot.findTeamProp();
//        teamBot.moveStraightWithEncoders(1, 10);
//
//        if(posOfTag == RobotClass.Position.LEFT){
//            teamBot.moveStraightWithEncoders(0.25, -10);
//            teamBot.gyroTurning(90);
//            teamBot.moveStraightWithEncoders(0.25, 15);
//            teamBot.moveStraightWithEncoders(-0.5, -60);
//            teamBot.gyroTurning(0);
//            teamBot.moveStraightWithEncoders(0.5, -5);
//            teamBot.gyroTurning(-90);
//            teamBot.moveWithoutEncoders(0.5, 0.5, 1500);
//
//        }else if(posOfTag == RobotClass.Position.CENTER){
//            teamBot.moveStraightWithEncoders(0.6,-38);
//            teamBot.gyroTurning(-90);
//            teamBot.moveWithoutEncoders(0.5, 0.5, 2000);
//
//        }else if(posOfTag == RobotClass.Position.RIGHT){
//            teamBot.moveStraightWithEncoders(0.25, -10);
//            teamBot.gyroTurning(-90);
//            teamBot.moveStraightWithEncoders(0.25, 10);
//            teamBot.moveStraightWithEncoders(0.5, -15);
//            teamBot.strafing(RobotClass.Direction.RIGHT, 0.5, 1500);
//            teamBot.moveStraightWithEncoders(0.5, 15);
//            teamBot.gyroTurning(-90);
//            teamBot.moveWithoutEncoders(0.5, 0.5, 500);
//        }
//
//        teamBot.clawRotator.setPosition(0.65);
//        teamBot.claw.setPosition(0);
//
//        sleep(1500);
//
//        teamBot.clawRotator.setPosition(0.25);
//
//        teamBot.moveStraightWithEncoders(0.5, -20);
//
//        teamBot.gyroTurning(-135);
//        teamBot.moveStraightWithEncoders(0.5, 50);
    }
}