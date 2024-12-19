
package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="TeleMode", group="Linear OpMode")

public class TeleModeV extends LinearOpMode {

    ElapsedTime runTime=new ElapsedTime();
    // Declare Drive Mtoros

    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;

    // Declare Rotation Motors
    DcMotor rightrot;
    DcMotor leftrot;

    // Declare Lift Motors
    DcMotorEx leftlift;
    DcMotorEx rightlift;

    AnalogInput distance;

    // Declare Diffy Servos


    // Declare Claw Servo
    Servo claw;
    Servo leftdiff;
    Servo rightdiff;

    Servo leftLED, rightLED;

    DigitalChannel clawswitch;
    BNO055IMU imu;
    int state = 0;
    int previousState = 0;
    boolean nighty = false;
    double gettime=0;
    double ledtime=0;
    double delaytime = 0;
    double driveSpeed = 0;

    double headingOffset = 0;
    double robotHeading  = 0;

    double ledValue = 0.27;

    boolean ledUpdate = false;

    double MAX_RANG = 520;
    double ADC_SOLUTION = 1023.0;
    double dis = 0.0;

    @Override
    public void runOpMode() {

        // Init Drive Motors
        frontLeftMotor  = hardwareMap.get(DcMotor.class, "fl");
        backLeftMotor  = hardwareMap.get(DcMotor.class, "bl");
        frontRightMotor = hardwareMap.get(DcMotor.class, "fr");
        backRightMotor = hardwareMap.get(DcMotor.class, "br");

        distance = hardwareMap.get(AnalogInput.class, "distance");


        // Init Rotation Motors
        rightrot = hardwareMap.get(DcMotor.class, "rightrot");
        leftrot  = hardwareMap.get(DcMotor.class, "leftrot");

        // Init Lift Motors
        leftlift  = hardwareMap.get(DcMotorEx.class, "leftlift");
        rightlift  = hardwareMap.get(DcMotorEx.class, "rightlift");


        // Init Diffy Servos
        leftdiff = hardwareMap.get(Servo.class, "leftdiff");
        rightdiff = hardwareMap.get(Servo.class, "rightdiff");

        // Init Claw Servo
        claw = hardwareMap.get(Servo.class, "claw");

        leftLED = hardwareMap.get(Servo.class, "leftled");
        rightLED = hardwareMap.get(Servo.class, "rightled");


        clawswitch = hardwareMap.get(DigitalChannel.class, "clawswitch");

        // Set Motor Direction for Drive Train
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set Motor Directions for Rotation Motors
        leftrot.setDirection(DcMotor.Direction.REVERSE);
        rightrot.setDirection(DcMotor.Direction.FORWARD);

        // Set Motor Directions for Lift Motors
        leftlift.setDirection(DcMotor.Direction.REVERSE);
        rightlift.setDirection(DcMotor.Direction.FORWARD);

        // Set Diffy Servos Init Position

        // comp comment
        // Set Claw Servo Init Position
//        clawClose();
//        diffyScore();

        //Set motors to brake Mode
        leftrot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightrot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftlift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightlift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftrot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightrot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftlift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightlift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // comp comment
//        leftrot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightrot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftlift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightlift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftrot.setTargetPosition(0);
        rightrot.setTargetPosition(0);
        leftlift.setTargetPosition(0);
        rightlift.setTargetPosition(0);

        leftrot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightrot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftrot.setPower(0);
        rightrot.setPower(0);
        leftlift.setPower(0);
        rightlift.setPower(0);

        clawswitch.setMode(DigitalChannel.Mode.INPUT);


        // Adjust the orientation parameters to match your robot


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        // Without this, data retrieving from the IMU throws an exception

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        leftlift.setTargetPositionTolerance(10);
        rightlift.setTargetPositionTolerance(10);

//        setLED(0.45);
//        sleep(1000);
//        setLED(0);


        telemetry.addData("Robot Ready", "");
        telemetry.addData("LED", ledValue);
        telemetry.update();

        waitForStart();
        runTime.reset();
        headingOffset = -imu.getAngularOrientation().firstAngle;




        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            switch (state){
                case 0: //home state
                    clawClose();
                    diffyScore();
                    rotateLift(0,0);
                    extendLift(0,0);
                    break;

                case 1: // collecting state
                    if(leftlift.getCurrentPosition()>1500){ //1950
                        if (gamepad1.right_bumper == true){
                            clawClose();
                        } else {
                            clawOpen();
                        }
                    }


                    if(gamepad1.left_trigger>0.1 && gamepad1.left_trigger < 0.5){
//                        diffyLeft();
                    } else if(gamepad1.right_trigger>0.1 && gamepad1.right_trigger < 0.5){
//                        diffyRight();
                    } else if (gamepad1.left_trigger > 0.5){
                        nighty = true;
                        diffyNightyLeft();
                    }else if (gamepad1.right_trigger > 0.5){
                        nighty = true;
                        diffyNightyRight();
                    }else{
                        if(leftlift.getCurrentPosition()>1450){
                            nighty = false;
                            diffyCollect();
                        }
                    }

                    if(leftlift.getCurrentPosition()>1450 && leftlift.getCurrentPosition()<1550){
                        extendLift(1550,0);
                    } else{
                        extendLift(1550,1);
                    }
                    break;

                case 2: // retracting state
                    clawClose();
                    if(nighty){
                        delaytime = 600;
                    }else{
//                        delaytime = 400;
                    }

                    if(runTime.milliseconds()-gettime>delaytime){
                        if(leftlift.getCurrentPosition()>-5 && leftlift.getCurrentPosition()<5){
                            extendLift(0,0);
                        } else{
                            extendLift(0,1);
                        }
                        diffyScore();
                    }else{
                        diffyCollect();
                    }






                    break;
                case 3: // extension state
                    clawOpen();
                    diffyTravel();
                    state = 1;

                    break;
                case 4: // rotate up
                    if (gamepad1.right_bumper == true){
                        clawOpen();
                    } else {
                        clawClose();
                    }
                    rotateLift(1450,1);


                    if(leftrot.getCurrentPosition()>1350){
                        extendLift(2700,1);
                    }
                    if(leftlift.getCurrentPosition()>2000){
                        diffyScore();
                    }

                    break;
                case 5: //rotate down
                    diffyScore();
//
//                    if(leftlift.getCurrentPosition()> 3500){
//                        rotateLift(1075,0.5);
//                    }

                    if(leftlift.getCurrentPosition()>-10 && leftlift.getCurrentPosition()<10){
                        extendLift(0,0);
                    } else {
//                        if(leftrot.getCurrentPosition()<=1075){
                            extendLift(0,1);
                        //}

                    }

                    if(leftrot.getCurrentPosition()>-20 && leftrot.getCurrentPosition()<20){
                        rotateLift(0,0);
                    } else{
                        if(leftlift.getCurrentPosition()<200){
                            rotateLift(0,0.75);
                        }
                    }
                     /*
                     if limit switch turn off motors stop and reset run to position
                     */

                    break;
                case 6:
                    diffyTravel();
                    sleep(100);
                    clawOpen();
//                    sleep(250);
                    state = previousState;
                    break;
                case 7:
                    rotateLift(1450,1);
                    if(leftrot.getCurrentPosition() >1350){
                        extendLift(800,1);
                    }
                    break;
                case 8:
                    extendLift(0,1);
                    if(leftlift.getCurrentPosition() <100){
                        rotateLift(0,1);
                    }
                    break;
                case 9: // collect specimen
                    if (gamepad1.right_bumper == true){
                        clawClose();
                    } else {
                        clawOpen();
                    }
                    rotateLift(200,1);
                    diffyTravel();

                    break;
                case 10: // rotate specimen
                    clawClose();
                    diffyScore();
                    rotateLift(1450, 1);
                    if(leftrot.getCurrentPosition() > 1350) {
                        extendLift(250, 1);
                    }
                    break;
                case 11: // score specimen
                    clawClose();
                    if(gamepad2.dpad_down){
                        extendLift(0, 1);
                        state = 10;
                    } else{
                        extendLift(1200, 1);
                    }

                    if(leftlift.getCurrentPosition() > 1150){
                        state = 12;
                    }
//                    else if(gamepad2.dpad_down){
//
//                    }
                    break;
                case 12:
                    clawOpen();
                    extendLift(0, 1);
                    if(leftlift.getCurrentPosition() < 50){
                        rotateLift(0,0.75);
                    }
                    break;
                default:
                    break;
            }
            if(gamepad2.left_trigger < 0.1) {
                if (gamepad2.dpad_right) {
                    state = 3;
                }
                if (clawPressed() && state == 1 || gamepad2.dpad_left) {
                    gettime = runTime.milliseconds();
                    state = 2;
                }
                if (gamepad2.dpad_up) {
                    state = 4;
                }
                if (gamepad2.dpad_down) { //!clawPressed() && state ==4 ||
                    state = 5;
                }
                if (gamepad1.a) {
                    previousState = state;
                    state = 6;
                }
                if (gamepad2.y) {
                    state = 7;
                }
                if (gamepad2.a) {
                    state = 8;
                }
            } else {
                if(gamepad2.dpad_right && gamepad2.left_trigger > 0.1) {
                    clawOpen();
                    state = 9;

                }
                if(clawPressed() && state ==9 ){

                    state=10;
                }
                if(gamepad2.dpad_up && gamepad2.left_trigger > 0.1) {

                    state = 11;

                }
            }


            if(gamepad2.left_trigger > 0.1){
                setLED(0.7);
            } else{
                setLED(0.1);
            }


            // if(runTime.seconds() > 5){
            //     setLED(0.3);
            // }
            // if(clawPressed() && ledUpdate ==false){
            //   ledtime = runTime.milliseconds();
            //   setLED(0.5);
            //   if(runTime.milliseconds()-ledtime>0.5){
            //       setLED(0);
            //       ledUpdate = true;
            //   }
            // }

            // if(!clawPressed()){
            //     ledUpdate = false;
            // }


/*

FIELD CENTRIC DRIVE CODE

*/
            if(leftlift.getCurrentPosition() > 500){
                driveSpeed = 0.5;
            }else{
                driveSpeed = 1;
            }

            double y = -gamepad1.left_stick_y * driveSpeed; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * driveSpeed;
            double rx = gamepad1.right_stick_x * driveSpeed;



            // Read inverse IMU heading, as the IMU heading is CW positive
            double botHeading = -imu.getAngularOrientation().firstAngle - headingOffset;

            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            if(gamepad1.start){
                headingOffset= -imu.getAngularOrientation().firstAngle;
            }
//            dis = distance.getVoltage() * MAX_RANG / ADC_SOLUTION;

            // Show the elapsed game time and wheel power.
//            telemetry.addData("Left Current", leftlift.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("Right Current", rightlift.getCurrent(CurrentUnit.AMPS));
//              telemetry.addData("Right Extend", rightlift.getCurrentPosition());
//              telemetry.addData("Left Extend", leftlift.getCurrentPosition());
//            telemetry.addData("distance",distance.getVoltage());
//
//            telemetry.addData("dis", dis);
////            telemetry.addData("claw Switch", clawswitch.getState());
//
            telemetry.addData("nighty", nighty);

//            telemetry.addData("angle", -imu.getAngularOrientation().firstAngle);
//            telemetry.addData("leftrot", leftrot.getCurrentPosition());

            telemetry.update();
        }
    }


    void clawOpen(){
        claw.setPosition(0.16);
    }
    void clawClose(){
        claw.setPosition(0);
    }
    void diffyScore(){
        rightdiff.setPosition(0.595);
        leftdiff.setPosition(0.600);
    }
    void diffyCollect(){
        rightdiff.setPosition(0.705);
        leftdiff.setPosition(0.490);
    }
    void diffyTravel(){
        rightdiff.setPosition(0.645);
        leftdiff.setPosition(0.550);
    }
    void diffyLeft(){
        rightdiff.setPosition(0.69);
        leftdiff.setPosition(0.475);
    }
    void diffyRight(){
        rightdiff.setPosition(0.75);
        leftdiff.setPosition(0.535);
    }

    void diffyNightyRight(){
        rightdiff.setPosition(0.765);
        leftdiff.setPosition(0.530);
    }

    void diffyNightyLeft(){
        rightdiff.setPosition(0.645);
        leftdiff.setPosition(0.430);
    }
    void rotateLift(int position, double speed){
        leftrot.setTargetPosition(position);
        rightrot.setTargetPosition(position);
        leftrot.setPower(speed);
        rightrot.setPower(speed);

    }
    void extendLift(int position, double speed){
        leftlift.setTargetPosition(position);
        rightlift.setTargetPosition(position);
        leftlift.setPower(speed);
        rightlift.setPower(speed);
    }

    void setLED(double value){
        leftLED.setPosition(value);
        rightLED.setPosition(value);
    }

    boolean clawPressed(){
        return(clawswitch.getState());



    }


}
