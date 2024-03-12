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
public class BlueBackstage extends LinearOpMode {
    //Instantiate robot class
    RobotClass teamBot = new RobotClass(this);
    
    int parkingPosition  = 1;
    
    public void runOpMode() throws InterruptedException {
        //initialize robot
        teamBot.init(hardwareMap);
        
        waitForStart();
        
        teamBot.moveStraightWithEncoders(0.6, 72);
        RobotClass.Position posOfTag = teamBot.findTeamProp();
        teamBot.clawRotator.setPosition(0.5);
//        teamBot.moveStraightWithEncoders(1, 10);
        
        if(posOfTag == RobotClass.Position.LEFT){
            teamBot.gyroTurning(90);
            teamBot.moveStraightWithEncoders(0.25, 15);
            teamBot.moveStraightWithEncoders(0.5, -18);
            teamBot.gyroTurning(135);
            teamBot.moveStraightWithEncoders(0.5, 50);
            teamBot.gyroTurning(45);
            teamBot.moveStraightWithEncoders(0.5, 50);
            teamBot.gyroTurning(90);
            teamBot.moveWithoutEncoders(0.5, 0.5, 1500);
            
        }else if(posOfTag == RobotClass.Position.CENTER){
            teamBot.moveStraightWithEncoders(1, 15);
            teamBot.moveStraightWithEncoders(0.6,-25);
            teamBot.gyroTurning(45);
            teamBot.moveStraightWithEncoders(1, 45);
            teamBot.gyroTurning(90);
            teamBot.moveWithoutEncoders(0.5, 0.5, 1500);
            
        }else if(posOfTag == RobotClass.Position.RIGHT){
            teamBot.gyroTurning(-90);
            teamBot.moveStraightWithEncoders(0.25, 15);
            teamBot.moveStraightWithEncoders(0.5, -50);
            teamBot.gyroTurning(0);
            teamBot.moveStraightWithEncoders(0.5, 35);
            teamBot.gyroTurning(90);
            teamBot.moveWithoutEncoders(0.5, 0.5, 1500);
        }
        
        
        teamBot.clawRotator.setPosition(0.65);
        teamBot.claw.setPosition(0);
        
        sleep(1500);
        
        teamBot.clawRotator.setPosition(0.25);
        sleep (500);
        
        teamBot.moveStraightWithEncoders(0.5, -20);
        
        if (posOfTag == RobotClass.Position.RIGHT) {
            teamBot.gyroTurning(180);
            teamBot.moveStraightWithEncoders(0.5, 80);
        }else{
        teamBot.gyroTurning(135);
        teamBot.moveStraightWithEncoders(0.5, 45);
        }
    }
}