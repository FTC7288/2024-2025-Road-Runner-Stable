package com.example.mmt;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class mmt {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();








        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-5, 61, Math.toRadians(90)))
//                .stopAndAdd(mech.rotate(1450)) // Lift Up
                .setTangent(Math.toRadians(135))
                .lineToYLinearHeading(49,Math.toRadians(225)) // Drive to front of bucket
//                .stopAndAdd(mech.lift(2700))
                .setTangent(180)
                .waitSeconds(0.25)
                .lineToX(51,new TranslationalVelConstraint(5)) // Drive to bucket 52
//                .stopAndAdd(mech.claw("open")) // score pre loaded
                .waitSeconds(.25)
                .lineToX(49)

//                .stopAndAdd(mech.lift(0))
                .splineToLinearHeading(new Pose2d(44,46, Math.toRadians(270)), Math.toRadians(270),
                        new TranslationalVelConstraint(8))
//                .stopAndAdd(mech.rotate(0))/
                .waitSeconds(0.5)
//                .stopAndAdd(mech.diffy("down"))
//                .stopAndAdd(mech.lift(1000)) // grab 1st block
                .waitSeconds(1)
//                .stopAndAdd(mech.claw("close"))
                .waitSeconds(0.5)
//                .stopAndAdd(mech.diffy("up"))
//                .stopAndAdd(mech.lift(0))
                .setTangent(45)
                .lineToYLinearHeading(47, Math.toRadians(225), new TranslationalVelConstraint(5))
//                .stopAndAdd(mech.rotate(1450))
                .waitSeconds(0.5)
//                .stopAndAdd(mech.lift(2700))
                .waitSeconds(.25)
                .lineToX(51, new TranslationalVelConstraint(5))

//                .stopAndAdd(mech.claw("open"))
                .waitSeconds(.25)
//                .stopAndAdd(mech.diffy("travel"))
                .lineToX(49)
//                .stopAndAdd(mech.lift(0))

                .setTangent(180)
                .splineToLinearHeading(new Pose2d(55,44, Math.toRadians(270)), Math.toRadians(270),
                        new TranslationalVelConstraint(8))
//                .stopAndAdd((mech.rotate(0)))
                .waitSeconds(0.5)
//                .stopAndAdd(mech.diffy("down"))
//                .stopAndAdd(mech.lift(1000))
                .waitSeconds(1)
//                .stopAndAdd(mech.claw("close"))
                .waitSeconds(0.5)


//                .stopAndAdd(mech.diffy("up"))
//                .stopAndAdd(mech.lift(0))
//                .stopAndAdd(mech.rotate(1450))
                .setTangent(90)
                .splineToLinearHeading(new Pose2d(45,48, Math.toRadians(225)), Math.toRadians(225),
                        new TranslationalVelConstraint(10))
//                .stopAndAdd(mech.lift(2700))
                .lineToX(51, new TranslationalVelConstraint(5))

//                .stopAndAdd(mech.claw("open"))
                .waitSeconds(.25)
                .lineToX(45)
//                .stopAndAdd(mech.lift(0))
                .waitSeconds(0.5)
//                        .waitSeconds(1)
                        .splineToLinearHeading(new Pose2d(35,15, Math.toRadians(270)), Math.toRadians(270), new TranslationalVelConstraint(10))
//                        .stopAndAdd(mech.rotate(0))
//                        .stopAndAdd(mech.claw("close"))
//                        .waitSeconds(2)

//                        .turn((Math.toRadians(180)))
//                .splineToLinearHeading(new Pose2d(46,25, Math.toRadians(0)), Math.toRadians(0), new TranslationalVelConstraint(10))
//                        .stopAndAdd(mech.lift(0))
//                        .stopAndAdd(mech.lift(0))
//                        .lineToX(50, new TranslationalVelConstraint(10))
//                        .waitSeconds(2)
//                        .stopAndAdd(mech.rotate(0))
//                        .turn(Math.toRadians(45))














                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}