package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;


@Config
@Autonomous (name = "Yellow Block Auto", group = "Autnomous")
public class YellowAuto extends LinearOpMode {
    AnalogInput distance;


    @Override
    public void runOpMode(){
        distance = hardwareMap.get(AnalogInput.class, "distance");
        Pose2d intialPos = new Pose2d(35, 61, Math.toRadians(180));
//        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, intialPos);
        Mechanism mech = new Mechanism(hardwareMap, 35,61,180);


        while(!isStopRequested() && !opModeIsActive()){
            telemetry.addData("Robot Ready", "");
            telemetry.addData("distance",distance.getVoltage());

            telemetry.update();
        }


        if (isStopRequested()) return;

        waitForStart();

        Actions.runBlocking(
                mech.drive.actionBuilder(intialPos)
                        .stopAndAdd(mech.rotate(1450)) // Lift Up
                        .setTangent(Math.toRadians(135))
                        .lineToYLinearHeading(49,Math.toRadians(225)) // Drive to front of bucket
                        .stopAndAdd(mech.lift(2500))
                        .setTangent(180)
                        .waitSeconds(0.25)
                        .lineToX(50,new TranslationalVelConstraint(5)) // Drive to bucket 52
                        .waitSeconds(0.25)
                        .stopAndAdd(mech.claw("open")) // score pre loaded
//                        .waitSeconds(.25)
                        .stopAndAdd(mech.diffy("travel"))
                        .waitSeconds(.5)
//                        .lineToX(49)

                        .stopAndAdd(mech.lift(0))
                        .splineToLinearHeading(new Pose2d(46,45, Math.toRadians(270)), Math.toRadians(270),
                                new TranslationalVelConstraint(6))
                        .stopAndAdd(mech.rotate(0))
                        .waitSeconds(0.5)
                        .stopAndAdd(mech.diffy("down"))
                        .stopAndAdd(mech.lift(1000)) // grab 1st block
                        .waitSeconds(1.25)
                        .stopAndAdd(mech.claw("close"))
                        .waitSeconds(0.5)
                        .stopAndAdd(mech.diffy("up"))
                        .stopAndAdd(mech.lift(0))
                        .setTangent(45)
                        .lineToYLinearHeading(48, Math.toRadians(225), new TranslationalVelConstraint(5))
                        .stopAndAdd(mech.rotate(1450))
                        .waitSeconds(0.5)
                        .stopAndAdd(mech.lift(2500))
                        .waitSeconds(.25)
                        .lineToX(51, new TranslationalVelConstraint(5))
                        .waitSeconds(0.5)
                        .stopAndAdd(mech.claw("open"))
//                        .waitSeconds(.25)
                        .stopAndAdd(mech.diffy("travel"))
                        .waitSeconds(0.5)
                        .stopAndAdd(mech.lift(0))

                        .setTangent(180)
                        .splineToLinearHeading(new Pose2d(54.5,44, Math.toRadians(270)), Math.toRadians(270),
                                new TranslationalVelConstraint(8))
                        .stopAndAdd((mech.rotate(0)))
                        .waitSeconds(0.5)
                        .stopAndAdd(mech.diffy("down"))
                        .stopAndAdd(mech.lift(1000))
                        .waitSeconds(1)
                        .stopAndAdd(mech.claw("close"))
                        .waitSeconds(0.5)


                        .stopAndAdd(mech.diffy("up"))
                        .stopAndAdd(mech.lift(0))
                        .stopAndAdd(mech.rotate(1450))
                        .setTangent(90)
                        .splineToLinearHeading(new Pose2d(45,48, Math.toRadians(225)), Math.toRadians(225),
                                new TranslationalVelConstraint(20))
                        .stopAndAdd(mech.lift(2500))
                        .waitSeconds(0.25)
                        .lineToX(49, new TranslationalVelConstraint(5))
                        .waitSeconds(0.5)
                        .stopAndAdd(mech.claw("open"))
//                        .waitSeconds(.25)
                        .stopAndAdd(mech.diffy("travel"))
                        .waitSeconds(0.5)
//                        .lineToX(45)
                        .stopAndAdd(mech.lift(0))
                        .waitSeconds(0.5)

                        .stopAndAdd(mech.diffy("down"))
                        .splineToLinearHeading(new Pose2d(46,23, Math.toRadians(0)), Math.toRadians(0), new TranslationalVelConstraint(20))
                        .stopAndAdd((mech.rotate(0)))
                        .waitSeconds(0.5)

                        .stopAndAdd(mech.lift(950))
                        .stopAndAdd(mech.diffy("nine"))
                        .waitSeconds(1)
                        .stopAndAdd(mech.claw("close"))
                        .waitSeconds(0.25)
                        .stopAndAdd(mech.diffy("down"))
                        .stopAndAdd(mech.lift(0))
                        .waitSeconds(0.5)
                        .stopAndAdd(mech.diffy("up"))
                        .stopAndAdd(mech.rotate(1450))
                        .splineToLinearHeading(new Pose2d(48,48, Math.toRadians(225)), Math.toRadians(225),
                                new TranslationalVelConstraint(18))
                        .stopAndAdd(mech.lift(2700))
                        .waitSeconds(0.25)
                        .lineToX(50, new TranslationalVelConstraint(5))
                        .waitSeconds(0.75)
                        .stopAndAdd(mech.claw("open"))
//                        .waitSeconds(.25)
                        .stopAndAdd(mech.diffy("travel"))
                        .waitSeconds(0.5)
                        .stopAndAdd(mech.lift(0))
                        .setTangent(0)
                        .turnTo(Math.toRadians(100))
                        .stopAndAdd(mech.rotate(0))
                        .stopAndAdd(mech.diffy("up"))
                        .waitSeconds(1)

                        .build());

    }
}
