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
public class RedAudience extends LinearOpMode {
    //Instantiate robot class
    RobotClass teamBot = new RobotClass(this);
    
    int parkingPosition  = 1;
    
    public void runOpMode() throws InterruptedException {
        //initialize robot
        teamBot.init(hardwareMap);
        
        waitForStart();
        
        teamBot.moveStraightWithEncoders(0.6, 82);
        RobotClass.Position posOfTag = teamBot.findTeamProp();
        
        
        if(posOfTag == RobotClass.Position.LEFT){
            teamBot.moveStraightWithEncoders(0.25, -10);
            teamBot.gyroTurning(90);
            teamBot.moveStraightWithEncoders(0.25, 15);
            teamBot.moveStraightWithEncoders(-0.5, -20);
            teamBot.gyroTurning(0);
            if(parkingPosition == 1){
                teamBot.moveStraightWithEncoders(0.5, 60);
                teamBot.gyroTurning(90);
                teamBot.moveStraightWithEncoders(0.5, -255);
            }
            
        }else if(posOfTag == RobotClass.Position.CENTER){
            teamBot.moveStraightWithEncoders(0.6,50);
            teamBot.gyroTurning(180);
            teamBot.moveStraightWithEncoders( 0.5, 25);
            if(parkingPosition == 1){
                teamBot.moveStraightWithEncoders(0.5, -30);
                teamBot.gyroTurning(90);
                teamBot.moveStraightWithEncoders(1, -255);
            }
        }else if(posOfTag == RobotClass.Position.RIGHT){
            teamBot.moveStraightWithEncoders(0.25, -10);
            teamBot.gyroTurning(-90);
            teamBot.moveStraightWithEncoders(0.25, 15);
            teamBot.moveStraightWithEncoders(0.25, -20);
            if(parkingPosition == 1){
                teamBot.gyroTurning(0);
                teamBot.moveStraightWithEncoders(0.5, 30);
                teamBot.gyroTurning(90);
                teamBot.moveStraightWithEncoders(1, -255);
            }
        }else{
            teamBot.moveStraightWithEncoders(0.25,-98);
            //Intake shoot
            teamBot.strafing(RobotClass.Direction.RIGHT,0.5,3000);
        }
    }
}