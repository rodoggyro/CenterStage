/*
 * This is a testing class for all auton path planning. Do NOT use this class for competition.
 * Starting positions are:
 *  - Blue Audience Side: (-35.5, 61, Math.toRadians(-90))
 *  - Blue Backstage Side: (12, 61, Math.toRadians(-90))
 *  - Red Audience Side: (-35.5, -61, Math.toRadians(90))
 *  - Red Backstage Side: (12, -61, Math.toRadians(90))
 */

package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    static boolean runRedAudience = false;
    static boolean runRedBackstage = false;
    static boolean runBlueAudience = true;
    
    public static void main(String[] args) {
        // Declare a MeepMeep instance
        // With a field size of 800 pixels
        MeepMeep meepMeep = new MeepMeep(600);
        
        if (runRedAudience) {
            RoadRunnerBotEntity RedAudience1 = new DefaultBotBuilder(meepMeep)
                    // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                    .setDimensions(16.5, 18)
                    // Option: Set theme. Default = ColorSchemeRedDark()
                    .setColorScheme(new ColorSchemeRedDark())
                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(new Pose2d(-35.5, -61, Math.toRadians(90)))
                                    //real value is 28.75
                                    .forward(30.5)
                                    .strafeLeft(14.5)
                                    .forward(-8)
                                    .splineTo(new Vector2d(-30.5, -11.0), Math.toRadians(90))
                                    .turn(Math.toRadians(90))
                                    .forward(82)
                                    .build()
                    );
            
            RoadRunnerBotEntity RedAudience2 = new DefaultBotBuilder(meepMeep)
                    // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                    .setDimensions(16.5, 18)
                    // Option: Set theme. Default = ColorSchemeRedDark()
                    .setColorScheme(new ColorSchemeRedDark())
                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(new Pose2d(-35.5, -61, Math.toRadians(90)))
                                    .forward(30.5)
                                    .forward(-5)
                                    .strafeRight(14.5)
                                    .turn(Math.toRadians(90))
                                    .lineTo(new Vector2d(-10, -30.5))
                                    .lineToSplineHeading(new Pose2d(-10, -11.0, Math.toRadians(180)))
                                    .lineTo(new Vector2d(51.5, -11.0))
                                    .build()
                    );
            
            RoadRunnerBotEntity RedAudience3 = new DefaultBotBuilder(meepMeep)
                    .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                    .setDimensions(16.5, 18)
                    .setColorScheme(new ColorSchemeRedDark())
                    .followTrajectorySequence(drive ->
                                    drive.trajectorySequenceBuilder(new Pose2d(-35.5, -61, Math.toRadians(90)))
                                            .forward(30.5)
                                            .forward(5)
                                            .forward(-10)
                                            .strafeRight(14.5)
                                            .splineTo(new Vector2d(38, -11.0), Math.toRadians(90))
                                            .lineTo(new Vector2d(51.5, -11.0))
                                            .build()
                    );
            
            // Set field image
            meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                    .setDarkMode(true)
                    // Background opacity from 0-1
                    .setBackgroundAlpha(1f)
                    .addEntity(RedAudience1)
                    .addEntity(RedAudience2)
                    .addEntity(RedAudience3)
                    .start();
        } else if (runRedBackstage) {
            RoadRunnerBotEntity RedBackstage3 = new DefaultBotBuilder(meepMeep)
                    .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                    .setDimensions(16.5, 18)
                    .setColorScheme(new ColorSchemeRedDark())
                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(new Pose2d(12, -61, Math.toRadians(90)))
                                    .forward(30.5)
                                    .forward(-5)
                                    .strafeLeft(10.5)
                                    .turn(Math.toRadians(90))
                                    .forward(-8)
                                    .lineToSplineHeading(new Pose2d(51.5, -58, Math.toRadians(180)))
                                    .build()
                    );
            
            RoadRunnerBotEntity RedBackstage2 = new DefaultBotBuilder(meepMeep)
                    .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                    .setDimensions(16.5, 18)
                    .setColorScheme(new ColorSchemeRedDark())
                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(new Pose2d(12, -61, Math.toRadians(90)))
                                    .forward(30.5)
                                    .forward(4.5)
                                    .forward(-10)
                                    .lineToSplineHeading(new Pose2d(51.5, -58, Math.toRadians(180)))
                                    .build()
                    );
            
            RoadRunnerBotEntity RedBackstage1 = new DefaultBotBuilder(meepMeep)
                    .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                    .setDimensions(16.5, 18)
                    .setColorScheme(new ColorSchemeRedDark())
                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(new Pose2d(12, -61, Math.toRadians(90)))
                                    .forward(30.5)
                                    .strafeRight(10.5)
                                    .forward(-10)
                                    .lineToSplineHeading(new Pose2d(51.5, -58, Math.toRadians(180)))
                                    .build()
                    );
            
            meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                    .setDarkMode(true)
                    // Background opacity from 0-1
                    .setBackgroundAlpha(1f)
                    .addEntity(RedBackstage3)
                    .addEntity(RedBackstage2)
                    .addEntity(RedBackstage1)
                    .start();
        } else if (runBlueAudience) {
            RoadRunnerBotEntity BlueAudience3 = new DefaultBotBuilder(meepMeep)
                    .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                    .setDimensions(16.5, 18)
                    .setColorScheme(new ColorSchemeRedDark())
                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(new Pose2d(-35.5, 61, Math.toRadians(-90)))
                                    .forward(30.5)
                                    .forward(5)
                                    .forward(-10)
                                    .strafeRight(14.5)
                                    .lineTo(new Vector2d(-50, 11.0))
                                    .lineToSplineHeading(new Pose2d(51.5, 11.0, Math.toRadians(180)))
                                    .build()
                    );
            
            RoadRunnerBotEntity BlueAudience2 = new DefaultBotBuilder(meepMeep)
                    // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                    .setDimensions(16.5, 18)
                    // Option: Set theme. Default = ColorSchemeRedDark()
                    .setColorScheme(new ColorSchemeRedDark())
                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(new Pose2d(-35.5, 61, Math.toRadians(-90)))
                                    .forward(30.5)
                                    .forward(-5)
                                    .strafeRight(14.5)
                                    .turn(Math.toRadians(-90))
                                    .lineTo(new Vector2d(-10, 30.5))
                                    .lineToSplineHeading(new Pose2d(-10, 11.0, Math.toRadians(180)))
                                    .lineTo(new Vector2d(51.5, 11.0))
                                    .build()
                    );
            RoadRunnerBotEntity BlueAudience1 = new DefaultBotBuilder(meepMeep)
                    // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                    .setDimensions(16.5, 18)
                    // Option: Set theme. Default = ColorSchemeRedDark()
                    .setColorScheme(new ColorSchemeRedDark())
                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(new Pose2d(-35.5, 61, Math.toRadians(-90)))
                                    //real value is 28.75
                                    .forward(30.5)
                                    .forward(-5)
                                    .strafeLeft(14.5)
                                    .turn(Math.toRadians(90))
                                    .forward(-10)
                                    .splineTo(new Vector2d(-30.5, 11.0), Math.toRadians(180))
                                    .forward(82)
                                    .build()
                    );
//
            
            meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                    .setDarkMode(true)
                    // Background opacity from 0-1
                    .setBackgroundAlpha(1f)
                    .addEntity(BlueAudience3)
                    .addEntity(BlueAudience2)
                    .addEntity(BlueAudience1)
                    .start();
        }
    }
}