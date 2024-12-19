package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Mechanism {
    private DcMotorEx leftlift, rightlift, leftrot, rightrot;
    private Servo leftdiff, rightdiff, claw;
    private AnalogInput distance;
     SparkFunOTOSDrive drive;
     Pose2d intialPos;


//    private double clawPos, diffyLeftPos, diffyRightPos;

    private int liftPos, rotatePos;

    public Mechanism(HardwareMap hardwareMap, int x, int y, int angle) {
        intialPos = new Pose2d(x, y, Math.toRadians(angle));
        drive = new SparkFunOTOSDrive(hardwareMap, intialPos);
        distance = hardwareMap.get(AnalogInput.class, "distance");

        leftlift = hardwareMap.get(DcMotorEx.class, "leftlift");
        rightlift = hardwareMap.get(DcMotorEx.class, "rightlift");
        leftrot = hardwareMap.get(DcMotorEx.class, "leftrot");
        rightrot = hardwareMap.get(DcMotorEx.class, "rightrot");

        claw = hardwareMap.get(Servo.class, "claw");
        leftdiff = hardwareMap.get(Servo.class, "leftdiff");
        rightdiff = hardwareMap.get(Servo.class, "rightdiff");

        claw = hardwareMap.get(Servo.class, "claw");

        leftdiff = hardwareMap.get(Servo.class, "leftdiff");
        rightdiff = hardwareMap.get(Servo.class, "rightdiff");

//            clawswitch = hardwareMap.get(DigitalChannel.class, "clawswitch");

        // Set Motor Directions for Rotation Motors
        leftrot.setDirection(DcMotorEx.Direction.REVERSE);
        rightrot.setDirection(DcMotorEx.Direction.FORWARD);

        // Set Motor Directions for Lift Motors
        leftlift.setDirection(DcMotorEx.Direction.REVERSE);
        rightlift.setDirection(DcMotorEx.Direction.FORWARD);

        leftrot.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightrot.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftlift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightlift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftrot.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightrot.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftlift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightlift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        leftrot.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightrot.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftlift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightlift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftrot.setTargetPosition(0);
        rightrot.setTargetPosition(0);
        leftlift.setTargetPosition(0);
        rightlift.setTargetPosition(0);

        leftrot.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightrot.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        leftlift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightlift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        leftrot.setPower(0);
        rightrot.setPower(0);
        leftlift.setPower(0);
        rightlift.setPower(0);

        claw.setPosition(0);

        rightdiff.setPosition(0.595);
        leftdiff.setPosition(0.600);

    }



    public Action distanceDrive(double dis){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (distance.getVoltage() < dis){

                    drive.rightFront.setPower(0);
                    drive.rightBack.setPower(0);
                    drive.leftFront.setPower(0);
                    drive.leftBack.setPower(0);
                    return false;
                }else{

                    drive.rightFront.setPower(-0.25);
                    drive.rightBack.setPower(-0.25);
                    drive.leftFront.setPower(-0.25);
                    drive.leftBack.setPower(-0.25);
                    return true;
                }
//
            }
        };
    }
    public Action rotateSlow(int rotatePos) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                double pos = leftrot.getCurrentPosition();
                // rotate down
                if(pos> rotatePos){
                    leftrot.setTargetPosition(rotatePos);
                    rightrot.setTargetPosition(rotatePos);
                    leftrot.setPower(0.5);
                    rightrot.setPower(0.5);
                }else{
                    leftrot.setTargetPosition(rotatePos);
                    rightrot.setTargetPosition(rotatePos);
                    leftrot.setPower(0.5);
                    rightrot.setPower(0.5);

                }

                return false;

            }
        };
    }

    public Action rotate(int rotatePos) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                double pos = leftrot.getCurrentPosition();
                // rotate down
                if(pos> rotatePos){
                    leftrot.setTargetPosition(rotatePos);
                    rightrot.setTargetPosition(rotatePos);
                    leftrot.setPower(0.5);
                    rightrot.setPower(0.5);
                }else{
                    leftrot.setTargetPosition(rotatePos);
                    rightrot.setTargetPosition(rotatePos);
                    leftrot.setPower(0.75);
                    rightrot.setPower(0.75);

                }

                return false;

            }
        };
    }


    public Action lift(int liftPos) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                double pos = leftlift.getCurrentPosition();

                // lift going down if current position is greater than the requested position
                if (pos > liftPos){
                    if (pos > (liftPos + 50)){

                        leftlift.setTargetPosition(liftPos);
                        rightlift.setTargetPosition(liftPos);
                        leftlift.setPower(1);
                        rightlift.setPower(1);
                        return false;
                    }else{
                        leftlift.setPower(0);
                        rightlift.setPower(0);
                        return false;
                    }

                }else{
                    if (pos < (liftPos - 50)){

                        leftlift.setTargetPosition(liftPos);
                        rightlift.setTargetPosition(liftPos);
                        leftlift.setPower(1);
                        rightlift.setPower(1);
                        return false;
                    }else{
                        return false;
                    }
                }
            }
        };

    }


    public Action claw(String pos) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (pos == "open") {
                    claw.setPosition( 0.16);
                } else if (pos == "close") {
                    claw.setPosition(0);
                }
                return false;
            }
        };


    }

    public Action diffy(String pos) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (pos == "down") {
                    rightdiff.setPosition(0.705);
                    leftdiff.setPosition(0.490);
                } else if (pos == "up") {
                    rightdiff.setPosition(0.595);
                    leftdiff.setPosition(0.600);
                } else if (pos == "travel"){
                    rightdiff.setPosition(0.645);
                    leftdiff.setPosition(0.550);
                } else if(pos == "nine"){
                    rightdiff.setPosition(0.645);
                    leftdiff.setPosition(0.430);
                }  else if(pos == "specimen"){
                    rightdiff.setPosition(0.645);
                    leftdiff.setPosition(0.550);
                }



                return false;
            }
        };


    }
}
