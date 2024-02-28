package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotClass;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous
public class RRBlueBackStage extends LinearOpMode {
    RobotClass.Position posOfTag;
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        RobotClass robot = new RobotClass(this);
        robot.init(hardwareMap);
        Pose2d startPose = new Pose2d(-35.5, -61, Math.toRadians(90));
        RobotClass teamBot = new RobotClass(this);
        teamBot.init(hardwareMap);
        drive.setPoseEstimate(startPose);
        
        waitForStart();
        
        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(3.7, 58, Math.toRadians(90)))
                .forward(26.75)
                .addDisplacementMarker(() -> {
                    posOfTag = robot.findTeamProp();
                })
                .build();
        drive.followTrajectorySequence(traj);
        
        if (posOfTag == RobotClass.Position.LEFT){
            TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading()))
                    .strafeRight(14)
                    .forward(-10)
                    .lineToSplineHeading(new Pose2d(drive.getPoseEstimate().getX() + 60, drive.getPoseEstimate().getY() - 10, Math.toRadians(-90)))
                    .build();
            
            drive.followTrajectorySequence(traj1);
        }
    }
}
