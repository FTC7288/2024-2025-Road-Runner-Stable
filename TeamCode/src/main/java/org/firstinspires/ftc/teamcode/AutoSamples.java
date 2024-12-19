//package org.firstinspires.ftc.teamcode;
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.TranslationalVelConstraint;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//
//@Config
//@Autonomous (name = "Blue Auto Mode", group = "Autnomous")
//public class AutoSamples extends LinearOpMode {
//
//
//
//
//    @Override
//    public void runOpMode(){
//        Pose2d intialPos = new Pose2d(35, 61, Math.toRadians(180));
//        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, intialPos);
//        Mechanism mech = new Mechanism(hardwareMap);
//
//
//        while(!isStopRequested() && !opModeIsActive()){
//            telemetry.addData("Robot Ready", "");
//            telemetry.update();
//        }
//
//        if (isStopRequested()) return;
//
//        waitForStart();
//
//        Actions.runBlocking(
//                drive.actionBuilder(intialPos)
//                        .stopAndAdd(mech.rotate(1125))
//                        .setTangent(Math.toRadians(135))
//                        .lineToYLinearHeading(50,Math.toRadians(225))
//                        .stopAndAdd(mech.lift(3600))
//                        .setTangent(180)
//                        .lineToX(52)
//
//
//                        .stopAndAdd(mech.claw("open"))
//                        .waitSeconds(.25)
//                        .lineToX(48)
//                        .stopAndAdd(mech.lift(0))
//                        .stopAndAdd(mech.rotate(0))
//
//                        .splineToLinearHeading(new Pose2d(44,45, Math.toRadians(265)), Math.toRadians(265))
//                        .stopAndAdd(mech.diffy("down"))
//
//                        .stopAndAdd(mech.lift(1300))
//                        .waitSeconds(0.5)
//                        .stopAndAdd(mech.claw("close"))
//                        .waitSeconds(0.5)
//                        .stopAndAdd(mech.diffy("up"))
//                        .stopAndAdd(mech.lift(0))
//                        .splineToLinearHeading(new Pose2d(49,52, Math.toRadians(225)), Math.toRadians(225))
////                        .stopAndAdd(mech.rotate(1125))
////                        .waitSeconds(1)
////                        .stopAndAdd(mech.lift(3600))
////                        .lineToX(52)
////                        .waitSeconds(.5)
////                        .stopAndAdd(mech.claw("open"))
////                        .waitSeconds(.25)
////                        .lineToX(48)
////                        .stopAndAdd(mech.lift(0))
////                        .stopAndAdd(mech.rotate(0))
////                        .splineToLinearHeading(new Pose2d(60,45, Math.toRadians(270)), Math.toRadians(270))
//
//
//
//
//
////                        .splineToLinearHeading(new Pose2d(55,55, Math.toRadians(225)), Math.toRadians(225))
////
////                        .setTangent(45)
////                        .lineToYLinearHeading(28,0)
////                        .setTangent(0)
////                        .lineToX(40)
////                        .setTangent(45)
////                        .lineToYLinearHeading(56,180)
////                        .lineToYLinearHeading(55,180)
////                        .splineToLinearHeading(new Pose2d(35,12,Math.toRadians(180)), Math.toRadians(180))
//                        .build());
//
//    }
//}
