package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.RobotClass;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "test")
public class RedAudienceTest extends LinearOpMode {
    
    RobotClass.Position posOfTag;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-35.5, -61, Math.toRadians(90));
        RobotClass teamBot = new RobotClass(this);
        teamBot.init(hardwareMap);
        drive.setPoseEstimate(startPose);
        waitForStart();
        
        if (isStopRequested()) return;
        
        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(-35.5, -61, Math.toRadians(90)))
                .forward(26.75)
                .addDisplacementMarker(() -> {
                    posOfTag = teamBot.findTeamProp();
                })
                .build();
        drive.followTrajectorySequence(traj);
        
        if (posOfTag == RobotClass.Position.LEFT) {
            TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading()))
                    .strafeRight(14)
                    .forward(-8)
                    .splineTo(new Vector2d(-29.5, -9.0), Math.toRadians(-90))
                    .strafeRight(25)
                    .turn(Math.toRadians(90))
                    .lineTo(new Vector2d(51.5, -11.0))
                    .build();
            
            drive.followTrajectorySequence(traj1);
        } else if (posOfTag == RobotClass.Position.RIGHT) {
            TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading()))
                    .forward(-5)
                    .strafeLeft(14.5)
                    .turn(Math.toRadians(90))
                    .lineTo(new Vector2d(-42, -30.5))
                    .lineToSplineHeading(new Pose2d(-42, -50, Math.toRadians(180)))
//                    .lineTo(new Vector2d(51.5, -11.0))
                    .build();
            
            drive.followTrajectorySequence(traj1);
        } else if (posOfTag == RobotClass.Position.CENTER) {
            TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading()))
                    .forward(3)
                    .forward(-10)
                    .strafeRight(14.5)
                    .splineTo(new Vector2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY() - 35), Math.toRadians(90))
                    .lineTo(new Vector2d(51.5, -11.0))
                    .build();
            
            drive.followTrajectorySequence(traj1);
        }
        
        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}
