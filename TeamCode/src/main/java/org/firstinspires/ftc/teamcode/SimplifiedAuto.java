package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;


@Config
@Autonomous (name = "Simp Auto", group = "Autnomous")
public class SimplifiedAuto extends LinearOpMode {
    AnalogInput distance;


    @Override
    public void runOpMode(){
        distance = hardwareMap.get(AnalogInput.class, "distance");

//        Pose2d intialPos = new Pose2d(-5, 61, Math.toRadians(90));
//        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, intialPos);
        Mechanism mech = new Mechanism(hardwareMap, -5, 61, 90);
//        TrajectoryActionBuilder tab1 = mech.drive.actionBuilder(new Pose2d(-5, 61, Math.toRadians(90)))
//                .lineToY(55, new TranslationalVelConstraint(10));
//
//        TrajectoryActionBuilder tab2 = mech.drive.actionBuilder(new Pose2d(-5, 61, Math.toRadians(90)))
//                .lineToY(30, new TranslationalVelConstraint(10));
//
//        Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
//                .build();

        while(!isStopRequested() && !opModeIsActive()){
            telemetry.addData("Robot Ready", "");
            telemetry.update();
        }


        if (isStopRequested()) return;
        waitForStart();


//        Actions.runBlocking(new SequentialAction(
//                tab1,
//                new ParallelAction(
//                        tab2,
//                        new SequentialAction(
//                            mech.distanceDrive()
//                        ),
//                        mech.lift(0)
//                ), trajectoryActionCloseOut
//        ));
        Actions.runBlocking(
                mech.drive.actionBuilder(new Pose2d(-5, 61, Math.toRadians(90)))
                        .lineToY(40, new TranslationalVelConstraint(10))
                        .stopAndAdd(mech.distanceDrive(0.106))
                        .splineToConstantHeading(new Vector2d(-40,33), Math.toRadians(270))
                        .waitSeconds(1)

                        .build());



    }
}
