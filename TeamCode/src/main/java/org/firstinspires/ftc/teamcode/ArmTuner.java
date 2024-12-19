package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
//import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
@TeleOp
@Disabled
public class ArmTuner extends OpMode {
    private PIDController controller;
    public static double  p = 0.02, i = 0, d =0.0005;
    public static double f= 0.08;
    public static int target = 0;
    public final double ticks_in_degrees = 4560.32 / 180;

    private DcMotorEx leftrot, rightrot;
    private DcMotorEx rightlift, leftlift;

    int leftlift_offset = 0;
    int rightlift_offset = 0;

    @Override
    public void init(){
        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        leftrot = hardwareMap.get(DcMotorEx.class, "leftrot");
        rightrot = hardwareMap.get(DcMotorEx.class, "rightrot");

        leftlift = hardwareMap.get(DcMotorEx.class, "leftlift");
        rightlift = hardwareMap.get(DcMotorEx.class, "rightlift");

        controller.setTolerance(20);

        leftlift.setPower(0);
        rightlift.setPower(0);

        leftlift.setDirection(DcMotorEx.Direction.REVERSE);
        rightlift.setDirection(DcMotorEx.Direction.FORWARD);

        leftrot.setDirection(DcMotorEx.Direction.REVERSE);
        rightrot.setDirection(DcMotorEx.Direction.FORWARD);

//        leftlift.setCurrentAlert(6,CurrentUnit.AMPS);
//        rightlift.setCurrentAlert(6,CurrentUnit.AMPS);

    }

    @Override
    public void loop(){
        controller.setPID(p,i,d);
        int armPos = leftrot.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target/ticks_in_degrees)) * f;

        double power = pid + ff;
        leftrot.setPower(power);
        rightrot.setPower(power);
//        if(gamepad1.a) {
//            target = 1140;
//        }else{
//            target =100;
//        }


//        if(gamepad1.dpad_left){
//            leftlift.setTargetPosition(2950);
//            rightlift.setTargetPosition(2950);
//            leftlift.setPower(1);
//            rightlift.setPower(1);
//
//        }
//
//        if(gamepad1.dpad_right){
//            leftlift.setTargetPosition(0 + leftlift_offset);
//            rightlift.setTargetPosition(0 +rightlift_offset);
//            leftlift.setPower(1);
//            rightlift.setPower(1);
//        }
//
//        if(leftlift.getCurrent(CurrentUnit.AMPS)>6){
//            leftlift_offset = leftlift.getCurrentPosition();
//            leftlift.setTargetPosition(leftlift.getTargetPosition() + leftlift_offset);
//        }
//        if(rightlift.getCurrent(CurrentUnit.AMPS) > 6){
//            rightlift_offset = rightlift.getCurrentPosition();
//            rightlift.setTargetPosition(rightlift.getTargetPosition() + rightlift_offset);
//        }

        telemetry.addData("armPos", armPos);
        telemetry.addData("target", target);
        telemetry.addData("Left current", leftlift.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Right current", rightlift.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("liftPos", leftlift.getCurrentPosition());
        telemetry.addData("lifttarget", leftlift.getTargetPosition());
        telemetry.update();

    }
}
