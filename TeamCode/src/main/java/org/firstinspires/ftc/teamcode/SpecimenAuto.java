package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;


@Config
@Autonomous (name = "Specimen Auto", group = "Autnomous")
public class SpecimenAuto extends LinearOpMode {
    AnalogInput distance;


    @Override
    public void runOpMode(){
        distance = hardwareMap.get(AnalogInput.class, "distance");

        Pose2d intialPos = new Pose2d(-5, 61, Math.toRadians(90));
//        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, intialPos);
        Mechanism mech = new Mechanism(hardwareMap, -5,61,90);


        while(!isStopRequested() && !opModeIsActive()){
            telemetry.addData("Robot Ready", "");
            telemetry.update();
        }


        if (isStopRequested()) return;
        waitForStart();

        Actions.runBlocking(
                mech.drive.actionBuilder(intialPos)
                        .stopAndAdd(mech.rotate(1450)) // Lift Up
                        .stopAndAdd(mech.lift(100))
                        .afterTime(1,mech.lift(500))
                        .lineToY(38.5, new TranslationalVelConstraint(20)) // 38.5
                        .stopAndAdd(mech.distanceDrive(0.14))
                        .waitSeconds(0.5)
                        .stopAndAdd(mech.lift(1200))
                        .waitSeconds(0.5)
                        .stopAndAdd(mech.claw("open"))
                        .stopAndAdd(mech.lift(0))
                        .setReversed(false)

//                        .setTangent(Math.toRadians(90))
//                        .afterDisp(3,mech.rotate(0))
                        .splineToConstantHeading(new Vector2d(-40,33), Math.toRadians(270))
//                        .splineToLinearHeading(new Pose2d(-40,33, Math.toRadians(90)), Math.toRadians(270))
                          .stopAndAdd(mech.rotate(0))
                        .splineToConstantHeading(new Vector2d(-49,15), Math.toRadians(180))
//                        .setTangent(Math.toRadians(250))
//                        .splineToLinearHeading(new Pose2d(-47,15, Math.toRadians(90)), Math.toRadians(180))
                        .setTangent(Math.toRadians(90))
                        .lineToY(56, new TranslationalVelConstraint(40))

                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(new Pose2d(-60,15, Math.toRadians(90)), Math.toRadians(180))

                        .setTangent(Math.toRadians(90))
                        .lineToY(56, new TranslationalVelConstraint(60))
                        .lineToY(50, new TranslationalVelConstraint(60))
                        .stopAndAdd(mech.rotate(200))
                        .stopAndAdd(mech.diffy("specimen"))
                        .lineToY(61.5, new TranslationalVelConstraint(30))
//                        .waitSeconds(0.25)
                        .stopAndAdd(mech.claw("close"))
                        .waitSeconds(0.5)
                        .stopAndAdd(mech.rotateSlow(1450))
                        .stopAndAdd(mech.diffy("up"))
//                        .waitSeconds(5)
                        .setTangent(Math.toRadians(315))
                        .afterDisp(3, mech.lift(500))
                        .splineToLinearHeading(new Pose2d(3,41.5, Math.toRadians(90)), Math.toRadians(315), new TranslationalVelConstraint(40))
                        .stopAndAdd(mech.distanceDrive(0.14))
                        .waitSeconds(0.5)
//                        .stopAndAdd(mech.rotateSlow(1450))
//                        .waitSeconds(0.75)
//                        .setTangent(Math.toRadians(90))
//                        .lineToY(39, new TranslationalVelConstraint(10))
                        .stopAndAdd( mech.lift(1200))
                        .waitSeconds(0.5)
                        .stopAndAdd(mech.claw("open"))
                        .waitSeconds(0.5)
                        .stopAndAdd(mech.lift(200))
////                        .setTangent(0)
//                        .lineToYLinearHeading(55,Math.toRadians(90), new TranslationalVelConstraint(10))
//
//                        .stopAndAdd(mech.rotate(180))
//                        .stopAndAdd(mech.claw("close"))
//                        .waitSeconds(1)

                        .afterDisp(3,mech.rotate(200))
                        .afterDisp(3, mech.diffy("specimen"))
//                        .setTangent(Math.toRadians(135))
                        .splineToLinearHeading(new Pose2d(-48,52, Math.toRadians(90)), Math.toRadians(180))
                        .setTangent(90)
                        .lineToY(60, new TranslationalVelConstraint(20))
                        .stopAndAdd(mech.claw("close"))
                        .waitSeconds(0.5)
                        .stopAndAdd(mech.rotateSlow(1450))
                        .stopAndAdd(mech.diffy("up"))
//                        .waitSeconds(5)
                        .setTangent(Math.toRadians(315))
                        .afterDisp(3, mech.lift(500))
                        .splineToLinearHeading(new Pose2d(-2,42, Math.toRadians(90)), Math.toRadians(315), new TranslationalVelConstraint(40))
                        .stopAndAdd(mech.distanceDrive(0.14))
                        .waitSeconds(0.5)
//                        .stopAndAdd(mech.rotateSlow(1450))
//                        .waitSeconds(0.75)
//                        .setTangent(Math.toRadians(90))
//                        .lineToY(39, new TranslationalVelConstraint(10))
                        .stopAndAdd( mech.lift(1200))
                        .waitSeconds(0.5)
                        .stopAndAdd(mech.claw("open"))
                        .waitSeconds(0.5)
                        .stopAndAdd(mech.lift(200))
                        .stopAndAdd(mech.rotate(0))

//                        .setTangent(Math.toRadians(90))
//                        .lineToY(63)
//                        .stopAndAdd(mech.claw("close"))
//                        .waitSeconds(0.5)
//                        .stopAndAdd(mech.rotate(1125))
//                        .stopAndAdd(mech.diffy("up"))
////                        .waitSeconds(5)
//                        .setTangent(Math.toRadians(315))
//                        .afterDisp(3, mech.lift(800))
//                        .splineToLinearHeading(new Pose2d(5,42.5, Math.toRadians(90)), Math.toRadians(315))
//                        .stopAndAdd( mech.lift(1700))
//                        .waitSeconds(0.5)
//                        .stopAndAdd(mech.claw("open"))
//                        .waitSeconds(0.5)
//                        .stopAndAdd(mech.lift(0))
//                        .waitSeconds(1)
////                        .setTangent(Math.toRadians(315))
////                        .splineToLinearHeading(new Pose2d(0,30, Math.toRadians(90)), Math.toRadians(315))











//                        .splineToLinearHeading(new Pose2d(55,55, Math.toRadians(225)), Math.toRadians(225))
//
//                        .setTangent(45)
//                        .lineToYLinearHeading(28,0)
//                        .setTangent(0)
//                        .lineToX(40)
//                        .setTangent(45)
//                        .lineToYLinearHeading(56,180)
//                        .lineToYLinearHeading(55,180)
//                        .splineToLinearHeading(new Pose2d(35,12,Math.toRadians(180)), Math.toRadians(180))
                        .build());



    }
}
