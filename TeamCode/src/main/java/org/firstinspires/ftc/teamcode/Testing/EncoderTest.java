package org.firstinspires.ftc.teamcode.Testing;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotClass;

@TeleOp
public class EncoderTest extends LinearOpMode {

    RobotClass robot = new RobotClass(this);

    public void runOpMode() {
        robot.init(hardwareMap);
        
        int[] encoderValues = {
                robot.frontLeft.getCurrentPosition(),
                robot.frontRight.getCurrentPosition(),
                robot.backLeft.getCurrentPosition(),
                robot.backRight.getCurrentPosition()
        };
        
        telemetry.addData("Front Left", "0");
        telemetry.addData("Front Right", "1");
        telemetry.addData("Back Left", "2");
        telemetry.addData("Back Right", "3");
        telemetry.update();
        
        waitForStart();

        while (!isStopRequested()) {
            if (gamepad1.a) {
                robot.resetEncoders();
                
                double throttle = -gamepad1.left_stick_y;
                double turn = gamepad1.right_stick_x;
                
                //setting power for forward-backward movement
                robot.frontLeft.setPower(throttle);
                robot.backLeft.setPower(throttle);
                robot.frontRight.setPower(throttle);
                robot.backRight.setPower(throttle);
                
                //setting up strafing
                if (gamepad1.left_bumper) {
                    robot.frontLeft.setPower(-0.75);
                    robot.backLeft.setPower(0.75);
                    robot.frontRight.setPower(-0.75);
                    robot.backRight.setPower(0.75);
                }else if (gamepad1.right_bumper) {
                    robot.frontLeft.setPower(0.75);
                    robot.backLeft.setPower(-0.75);
                    robot.frontRight.setPower(0.75);
                    robot.backRight.setPower(-0.75);
                }
                
                //setting power for turning
                robot.frontLeft.setPower(turn);
                robot.backLeft.setPower(turn);
                robot.frontRight.setPower(-turn);
                robot.backRight.setPower(-turn);
            } else if (gamepad1.b) {
                robot.frontLeft.setTargetPosition(1000);
                robot.frontRight.setTargetPosition(1000);
                robot.backLeft.setTargetPosition(1000);
                robot.backRight.setTargetPosition(1000);

                robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.frontLeft.setPower(0.25);
                robot.frontRight.setPower(0.25);
                robot.backLeft.setPower(0.25);
                robot.backRight.setPower(0.25);

                while (robot.backRight.isBusy()) {
                    telemetry.addData("Front Left", robot.frontLeft.getCurrentPosition());
                    telemetry.addData("Front Right", robot.frontRight.getCurrentPosition());
                    telemetry.addData("Back Left", robot.backLeft.getCurrentPosition());
                    telemetry.addData("Back Right", robot.backRight.getCurrentPosition());

                    telemetry.update();
                }

                robot.stopMotors();
            } else if (gamepad1.x) {
                double circumferenceCm = (double) 48 / 10;
                double ticksPerRotation = 2000;
                double ticksPerCm = ticksPerRotation / circumferenceCm;
                int target = (int) Math.round(40 * ticksPerCm);
                
                double minCorrectionPower = 0.2;
                double maxCorrectionPower = 1.0;
                
                //resetting
                robot.resetEncoders();
                robot.odowheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.odowheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                
                boolean run = true;
                while (run) {
                    telemetry.addData("Target", target);
                    telemetry.addData("Current", robot.odowheel.getCurrentPosition());
                    telemetry.update();
                    //using gyro
                    if (robot.odowheel.getCurrentPosition() >= target - 5 && robot.odowheel.getCurrentPosition() <= target + 5) {
                        robot.frontLeft.setPower(0);
                        robot.frontRight.setPower(0);
                        robot.backLeft.setPower(0);
                        robot.backRight.setPower(0);
                        sleep(500);
                        if (robot.odowheel.getCurrentPosition() >= target - 5 && robot.odowheel.getCurrentPosition() <= target + 5) {
                            run = false;
                        }
                    } else if (robot.odowheel.getCurrentPosition() >= target) {
                        if (robot.odowheel.getCurrentPosition() <= target + 100) {
                            robot.frontLeft.setPower(-minCorrectionPower);
                            robot.frontRight.setPower(-minCorrectionPower);
                            robot.backLeft.setPower(-minCorrectionPower);
                            robot.backRight.setPower(-minCorrectionPower);
                        } else {
                            robot.frontLeft.setPower(-maxCorrectionPower);
                            robot.frontRight.setPower(-maxCorrectionPower);
                            robot.backLeft.setPower(-maxCorrectionPower);
                            robot.backRight.setPower(-maxCorrectionPower);
                        }
                    } else if (robot.odowheel.getCurrentPosition() <= target) {
                        if (robot.odowheel.getCurrentPosition() >= target - 100) {
                            robot.frontLeft.setPower(minCorrectionPower);
                            robot.frontRight.setPower(minCorrectionPower);
                            robot.backLeft.setPower(minCorrectionPower);
                            robot.backRight.setPower(minCorrectionPower);
                            
                        } else {
                            robot.frontLeft.setPower(maxCorrectionPower);
                            robot.frontRight.setPower(maxCorrectionPower);
                            robot.backLeft.setPower(maxCorrectionPower);
                            robot.backRight.setPower(maxCorrectionPower);
                        }
                    }
                    robot.stopMotors();
                }
            }
            
            encoderValues[0] = robot.frontLeft.getCurrentPosition();
            encoderValues[1] = robot.frontRight.getCurrentPosition();
            encoderValues[2] = robot.backLeft.getCurrentPosition();
            encoderValues[3] = robot.backRight.getCurrentPosition();

            for(int value : encoderValues) {
                telemetry.addData("Encoder " + value, encoderValues[value]);
            }
            telemetry.update();
        }
    }
}
