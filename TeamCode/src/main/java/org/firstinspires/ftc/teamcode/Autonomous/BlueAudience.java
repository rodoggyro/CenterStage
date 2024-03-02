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
public class BlueAudience extends LinearOpMode {
    //Instantiate robot class
    RobotClass teamBot = new RobotClass(this);
    
    int parkingPosition  = 2;
    
    public void runOpMode() throws InterruptedException {
        //initialize robot
        teamBot.init(hardwareMap);
        
        waitForStart();
        
        teamBot.moveStraightWithEncoders(0.6, 72);
        RobotClass.Position posOfTag = teamBot.findTeamProp();
        teamBot.clawRotator.setPosition(0.25);
        
        if(posOfTag == RobotClass.Position.LEFT){
            teamBot.gyroTurning(90);
            teamBot.moveStraightWithEncoders(0.25, 18);
            teamBot.moveStraightWithEncoders(0.5, -20);
            teamBot.gyroTurning(0);
            teamBot.moveStraightWithEncoders(1, 60);
            teamBot.gyroTurning(90);
            teamBot.moveWithoutEncoders(1,1,2000);
        }else if(posOfTag == RobotClass.Position.CENTER){
            teamBot.moveStraightWithEncoders(0.5, 50);
            teamBot.gyroTurning(180);
            teamBot.moveStraightWithEncoders(0.5, 10);
            teamBot.moveStraightWithEncoders(0.5, -15);
            teamBot.gyroTurning(-90);
            teamBot.moveWithoutEncoders(-1,-1, 2000);
        }else if(posOfTag == RobotClass.Position.RIGHT){
            teamBot.gyroTurning(-90);
            teamBot.moveStraightWithEncoders(0.25, 15);
            teamBot.moveStraightWithEncoders(0.5, -18);
            teamBot.gyroTurning(0);
            teamBot.moveStraightWithEncoders(1, 60);
            teamBot.gyroTurning(90);
            teamBot.moveWithoutEncoders(-1,-1,2000);
        }
    }
}