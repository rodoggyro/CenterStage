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

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                robot.resetEncoders();
            } else if (gamepad1.b) {
                robot.robot.frontLeft.setTargetPosition(1000);
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
                            return;
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

            telemetry.addData("Front Left", robot.frontLeft.getCurrentPosition());
            telemetry.addData("Front Right", robot.frontRight.getCurrentPosition());
            telemetry.addData("Back Left", robot.backLeft.getCurrentPosition());
            telemetry.addData("Back Right", robot.backRight.getCurrentPosition());

            telemetry.update();
        }
    }
}
