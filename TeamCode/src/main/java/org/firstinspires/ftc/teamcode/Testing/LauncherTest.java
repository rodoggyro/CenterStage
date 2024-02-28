package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class LauncherTest extends LinearOpMode {
    Servo launcher;
    
    public void runOpMode() {
        launcher = hardwareMap.get(Servo.class, "launcher");
        
        launcher.setPosition(0);
        
        waitForStart();
        
        launcher.setPosition(1);
        
        sleep(5000);
    }
}
