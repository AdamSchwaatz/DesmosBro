package org.firstinspires.ftc.teamcode.Brobotix;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Adam on 2/17/2018.
 */

@TeleOp(name = "Mecanum Drive", group = "Adam")

public class MecanumDrive extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    MecanumBasebot robot = new MecanumBasebot();   // Use a Pushbot's hardware

    static final double liftSpeed = 1;
    static final double pushSpeed = 1;
    static final double handSpeed = 0.25;
    static final double handUp = 2.294;
    static final double handDown = 0.74;
    static final double ticksRevHD = 2240;
    static final double ticksCoreHex = 288;
    static final int WHEEL_DIAMETER_INCHES = 1;
    static final double SHAFT_DIAMETER_INCHES = 0.75;
    static final double VERTICAL_GEAR_REDUCTION = 2;
    static final int HORIZONTAL_GEAR_REDUCTION = 1;
    static final int HAND_GEAR_REDUCTION = 1;
    static final double PI = 3.1415;
    static final double COUNTS_PER_INCH_HORIZONTAL = (ticksRevHD * HORIZONTAL_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * PI);
    static final double COUNTS_PER_INCH_VERTICAL = (ticksCoreHex * VERTICAL_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * PI);
    static final double COUNTS_PER_INCH_HAND = (ticksCoreHex * HAND_GEAR_REDUCTION) / (SHAFT_DIAMETER_INCHES * PI);
    public int dumpPosition = 0;
    public int liftPosition = 0;
    public int spinDirection = 0;
    public int pushPosition = 0;
    public double handPosition = 0;
    public int lockPosition = 0;
    public boolean x1Pressed = false;
    public boolean x2Pressed = false;
    public boolean y1Pressed = false;
    public boolean y2Pressed = false;
    public boolean a1Pressed = false;
    public boolean a2Pressed = false;
    public boolean b1Pressed = false;
    public boolean b2Pressed = false;
    public boolean dpadUp2Pressed = false;
    public boolean dpadDown2Pressed = false;
    public boolean rightBumper1Pressed = false;
    public boolean rightBumper2Pressed = false;
    public boolean leftBumper2Pressed = false;
    public boolean pushGo = false;
    public boolean liftGo = false;
    public boolean handGo = false;
    public boolean liftEngaged = false;
    public boolean handEngaged = false;
    public boolean pushEngaged = false;
    public boolean pass = false;
    public double passTime = 0;
    public boolean dancing = false;
    public double danceTime = 0;
    public int mrroboto;


    public enum Status {
        Starting,
        Lifting,
        Pushing,
        Hand,
        LP,
        LH,
        PH,
        LPH,
        Dancing
    }

    Status status = Status.Starting;


//            Gamepad 1       Gampepad 2
//            x               x push-out-in
//            y unlock        y pass to self
//            a               a spinners-clock-counter-stop
//            b dump          b
//            leftJoystick    leftJoystick
//            x drive         x
//            y drive         y lift
//            rightJoystick   rightJoystick
//            x turn          x
//            y turn          y grabber
//            d-pad           d-pad
//            up push-out     up lift position up
//            down push-in    down lift position down
//            left            left
//            right           right


    @Override
    public void runOpMode() {
        //Initialize the robot
        robot.init(hardwareMap);

        mrroboto = hardwareMap.appContext.getResources().getIdentifier("mrroboto", "raw", hardwareMap.appContext.getPackageName());

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        runtime.reset();

        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.handMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.push.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.push.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.handMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Set the status
        Status status = Status.Starting;

        //Starting status
        while (status == Status.Starting && opModeIsActive()) {
            driving();
        }
        //Only lifting
        while (status == Status.Lifting && opModeIsActive()) {
            driving();
            robot.lift.setPower(liftSpeed);
        }
        //Only pushing
        while (status == Status.Pushing && opModeIsActive()) {
            driving();
            robot.push.setPower(pushSpeed);
        }
        //Only hand
        while (status == Status.Hand && opModeIsActive()) {
            driving();
            checkHand();
        }
        //Lifting, Pushing, and Hand
        while (status == Status.LPH && opModeIsActive()) {
            driving();
            robot.lift.setPower(liftSpeed);
            robot.push.setPower(pushSpeed);
            checkHand();
        }
        //Lifting and Pushing
        while (status == Status.LP && opModeIsActive()) {
            driving();
            robot.lift.setPower(liftSpeed);
            robot.push.setPower(pushSpeed);
        }
        //Lifting and Hand
        while (status == Status.LH && opModeIsActive()) {
            driving();
            robot.lift.setPower(liftSpeed);
            checkHand();
        }
        //Pushing and Hand
        while (status == Status.PH && opModeIsActive()) {
            driving();
            robot.push.setPower(pushSpeed);
            checkHand();
        }
        //Dancing
        while (status == Status.Dancing && opModeIsActive()){
            driving();
            robot.lift.setPower(liftSpeed);
            robot.push.setPower(pushSpeed);
            checkHand();
        }

        //Stop the motors at the end of the tele-op period
        robot.lift.setPower(0);
        robot.push.setPower(0);
        robot.handMotor.setPower(0);
        robot.leftFrontMotor.setPower(0);
        robot.rightFrontMotor.setPower(0);
        robot.leftRearMotor.setPower(0);
        robot.rightRearMotor.setPower(0);

    }

    void checkStatus() {
        //Check what the status of the robot is
        if(dancing){
            status = Status.Dancing;
        }else if (liftGo && pushGo && handGo) {
            status = Status.LPH;
        } else if (liftGo && pushGo) {
            status = Status.LP;
        } else if (pushGo && handGo) {
            status = Status.PH;
        } else if (liftGo && handGo) {
            status = Status.LH;
        } else if (liftGo) {
            status = Status.Lifting;
        } else if (pushGo) {
            status = Status.Pushing;
        } else if (handGo) {
            status = Status.Hand;
        } else {
            status = Status.Starting;
        }
    }
    void checkButtons() {
        //Check if the buttons are pressed
        //Intended to allow for onPress usage of buttons
        if (!gamepad1.x) {
            x1Pressed = false;
        }
        if (!gamepad2.x) {
            x2Pressed = false;
        }
        if (!gamepad1.y) {
            y1Pressed = false;
        }
        if (!gamepad2.y) {
            y2Pressed = false;
        }
        if (!gamepad1.a) {
            a1Pressed = false;
        }
        if (!gamepad2.a) {
            a2Pressed = false;
        }
        if (!gamepad1.b) {
            b1Pressed = false;
        }
        if (!gamepad2.b) {
            b2Pressed = false;
        }
        if(!gamepad2.dpad_up){
            dpadUp2Pressed = false;
        }
        if(!gamepad2.dpad_down){
            dpadDown2Pressed = false;
        }
        if(!gamepad1.right_bumper){
            rightBumper1Pressed = false;
        }
        if(!gamepad2.right_bumper){
            rightBumper2Pressed = false;
        }
        if(!gamepad2.left_bumper){
            leftBumper2Pressed = false;
        }
    }
    void checkMotors() {
        //Checks if the motors are done going to a position
        if (!robot.lift.isBusy()) {
            liftGo = false;
        }
        if (handGo && Math.abs(robot.potentiometer.getVoltage() - handPosition) < .1 ) {
            handGo = false;
        }
        if (!robot.push.isBusy()) {
            pushGo = false;
        }
    }
    void checkHand(){
        if(robot.potentiometer.getVoltage() > handPosition){
            robot.handMotor.setPower(-1);
        }else{
            robot.handMotor.setPower(1);
        }
    }
    void driving() {
        //Looped through continuously throughout the tele-op period
        checkStatus();
        checkButtons();
        checkMotors();
        dance();

        //Teletry for runtime, both gamepads, status, and the positions of hand, lift, and push motors
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Status", "Gamepad1: " + gamepad1.toString());
        telemetry.addData("Status", "Gamepad2: " + gamepad2.toString());
        telemetry.addData("Status", "Status: " + status);
        telemetry.addData("Status","Hand Position: " + robot.potentiometer.getVoltage());
        telemetry.addData("Status", "Hand Target: " + handPosition);
        telemetry.addData("Status","Lift Position: " + robot.lift.getCurrentPosition() / COUNTS_PER_INCH_VERTICAL);
        telemetry.addData("Status","Push Position: " + robot.push.getCurrentPosition() / COUNTS_PER_INCH_HORIZONTAL);
        telemetry.update();

        //Driving
        if(!(status == Status.Dancing)) {
            double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;
            robot.leftFrontMotor.setPower(v1);
            robot.rightFrontMotor.setPower(v2);
            robot.leftRearMotor.setPower(v3);
            robot.rightRearMotor.setPower(v4);
        }

        //Lift Control
        if (gamepad2.left_stick_y > 0 || gamepad2.left_stick_y < 0) {
            robot.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.lift.setPower(gamepad2.left_stick_y);
            liftEngaged = true;
        } else if (liftEngaged && gamepad2.left_stick_y == 0) {
            robot.lift.setPower(0);
            liftEngaged = false;
        }

        //Hand Control
//        if ((gamepad2.right_stick_y > 0 || gamepad2.right_stick_y < 0)) {
//            robot.handMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            robot.handMotor.setPower(gamepad2.right_stick_y*0.75);
//            handEngaged = true;
//        } else if (handEngaged && gamepad2.right_stick_y == 0) {
//            robot.handMotor.setPower(0);
//            handEngaged = false;
//        }
        //Hand Control
        if(gamepad2.right_stick_y < 0 && robot.potentiometer.getVoltage() < handUp - 0.2){
            handEngaged = true;
            robot.handMotor.setPower(gamepad2.right_stick_y*0.75);
        }else if(gamepad2.right_stick_y > 0 && robot.potentiometer.getVoltage() > handDown + 0.1){
            handEngaged = true;
            robot.handMotor.setPower(gamepad2.right_stick_y*0.75);
        }else{
            robot.handMotor.setPower(0);
        }

        //Hand
        if(gamepad2.dpad_up && !dpadUp2Pressed && !gamepad2.dpad_down){
            dpadUp2Pressed = true;
            handGo = true;
            handPosition = robot.potentiometer.getVoltage();
            if(handPosition <= handUp) {
                handPosition = handUp;
                handGo = true;
            }
        }
        if(gamepad2.dpad_down && !dpadDown2Pressed && !gamepad2.dpad_up){
            dpadDown2Pressed = true;
            handGo = true;
            handPosition = robot.potentiometer.getVoltage();
            if(handPosition >= handDown) {
                handPosition = handDown;
                handGo = true;
            }
        }

        //Pass to self
        if(gamepad2.right_bumper  && !rightBumper2Pressed){
            rightBumper2Pressed = true;
            liftGo = true;
            pass = true;
            robot.dumpHand.setPosition(0);
            dumpPosition = 0;
            passTime = runtime.seconds();
            moveVertical(-7);
        }
        if(pass && (runtime.seconds() - passTime) > 2){
            handGo = true;
            handPosition = 2;
            pass = false;
        }

        //Push Control
//        if (gamepad1.dpad_up || gamepad1.dpad_down) {
//            while (gamepad1.dpad_up) {
//                robot.handMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                robot.push.setPower(0.5);
//                pushEngaged = true;
//            }
//            while (gamepad1.dpad_down) {
//                robot.handMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                robot.push.setPower(-0.2);
//                pushEngaged = true;
//            }
//        }
//        if (!(gamepad1.dpad_up || gamepad1.dpad_down) && pushEngaged) {
//            robot.push.setPower(0);
//            pushEngaged = false;
//        }

        //Spinners
        if (gamepad2.a && !a2Pressed) {
            a2Pressed = true;
            if (spinDirection == 0) {
                robot.rightHand.setPosition(1);
                robot.leftHand.setPosition(1);
                spinDirection = 1;
            } else if (spinDirection == 1) {
                robot.rightHand.setPosition(0);
                robot.leftHand.setPosition(0);
                spinDirection = 2;
            } else if (spinDirection == 2) {
                robot.rightHand.setPosition(0.5);
                robot.leftHand.setPosition(0.5);
                spinDirection = 0;
            }
        }

        //Dump Hand
        if (gamepad1.b && !b1Pressed) {
            b1Pressed = true;
            if (dumpPosition == 0) {
                robot.dumpHand.setPosition(1);
                dumpPosition = 1;
            } else if (dumpPosition == 1) {
                robot.dumpHand.setPosition(0);
                dumpPosition = 0;
            }
        }

        //Unlock
        if (gamepad1.y && !y1Pressed) {
            y1Pressed = true;
            if (lockPosition == 0) {
                robot.lock.setPosition(0);
                lockPosition = 1;
            } else if (lockPosition == 1) {
                robot.lock.setPosition(0.5);
                lockPosition = 0;
            }
        }



        //Lift
//        if (gamepad2.y && !y2Pressed) {
//            y2Pressed = true;
//            liftGo = true;
//            if (liftPosition == 0) {
//                //9.5
//                moveVertical(-5);
//                liftPosition = -5;
//            } else if (liftPosition < 0) {
//                moveVertical(0);
//                liftPosition = 0;
//            }
//        }

        //Lift
//        if(gamepad2.dpad_up && !dpadUp2Pressed && !gamepad2.dpad_down){
//            dpadUp2Pressed = true;
//            liftGo = true;
//            if(liftPosition <= 0) {
//                moveVertical(liftPosition + 2);
//                liftPosition = liftPosition + 2;
//            }
//        }
//        if(gamepad2.dpad_down && !dpadDown2Pressed && !gamepad2.dpad_up){
//            dpadDown2Pressed = true;
//            liftGo = true;
//            if(liftPosition >= -7) {
//                moveVertical(liftPosition - 2);
//                liftPosition = liftPosition - 2;
//            }
//        }

        //Push Arm
        if (gamepad2.x && !x2Pressed) {
            x2Pressed = true;
            if (pushPosition == 0) {
                pushGo = true;
                moveHorizontal(6);
                pushPosition = 1;
            } else if (pushPosition == 1) {
                pushGo = true;
                moveHorizontal(0);
                pushPosition = 0;
            }
        }

        //Hand Drop
//        if (gamepad2.b && !b2Pressed) {
//            b2Pressed = true;
//            if (dropPosition == 0) {
//                moveHand(1);
//                liftPosition = 1;
//            } else if (liftPosition == 1) {
//                moveHand(0);
//                liftPosition = 0;
//            }
//        }
    }
    void moveVertical(double position) {
        //Intended to move the distance up in inches passed to the function
        // How far are we to move, in ticks instead of revolutions
        int liftDistance = -(int) Math.round(position * COUNTS_PER_INCH_VERTICAL);
        // Tell the motors where we are going
        robot.lift.setTargetPosition(liftDistance);
        //Run
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setPower(liftSpeed);
    }
    void moveHorizontal(double position) {
        //Intended to push the distance in inches passed to the function
        // How far are we to push, in ticks instead of revolutions
        int pushDistance = -(int) Math.round(position * COUNTS_PER_INCH_HORIZONTAL);
        // Tell the motors where we are going
        robot.push.setTargetPosition(pushDistance);
        //Run
        robot.push.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.push.setPower(pushSpeed);
    }
    void moveHand(double position) {
        //Intended to move the distance forward in inches passed to the function
        // How far are we to move, in ticks instead of revolutions
        int handDistance = (int) Math.round(position * COUNTS_PER_INCH_HAND);
        // Tell the motors where we are going
        robot.handMotor.setTargetPosition(handDistance);
        //Run
        robot.handMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.handMotor.setPower(handSpeed);
    }
    void dance(){
        //This makes the robot dance whenever we want it to when the
        //right bumper is pressed on gamepad1
        //It lifts and pushes on intervals of two seconds
        //and every second it moves the dump and lock servos as
        //well as changing the direction of the spinners

        //Dancing
        if(gamepad1.right_bumper && rightBumper1Pressed){
            rightBumper1Pressed = true;
            if(dancing){
                dancing = false;
                robot.rightHand.setPosition(0.5);
                robot.leftHand.setPosition(0.5);
                spinDirection = 0;
                robot.leftFrontMotor.setPower(0);
                robot.rightFrontMotor.setPower(0);
                robot.leftRearMotor.setPower(0);
                robot.rightRearMotor.setPower(0);
            }else{
                dancing = true;
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, mrroboto);
            }
            danceTime = runtime.seconds();

        }
        if(dancing){
            robot.leftFrontMotor.setPower(1);
            robot.rightFrontMotor.setPower(-1);
            robot.leftRearMotor.setPower(1);
            robot.rightRearMotor.setPower(-1);
            if(runtime.seconds() - danceTime < 1){
                robot.lock.setPosition(0);
                lockPosition = 1;
                robot.dumpHand.setPosition(1);
                dumpPosition = 1;
                robot.rightHand.setPosition(1);
                robot.leftHand.setPosition(1);
                spinDirection = 1;
                pushGo = true;
                moveHorizontal(6);
                pushPosition = 1;
            }else if(runtime.seconds() - danceTime < 2){
                robot.lock.setPosition(0.5);
                lockPosition = 0;
                robot.dumpHand.setPosition(0);
                dumpPosition = 0;
                robot.rightHand.setPosition(0);
                robot.leftHand.setPosition(0);
                spinDirection = 2;
                moveVertical(-5);
                liftPosition = -5;
            }else if(runtime.seconds() - danceTime < 3){
                robot.lock.setPosition(0);
                lockPosition = 1;
                robot.dumpHand.setPosition(1);
                dumpPosition = 1;
                robot.rightHand.setPosition(1);
                robot.leftHand.setPosition(1);
                spinDirection = 1;
                pushGo = true;
                moveHorizontal(0);
                pushPosition = 0;
            }else if(runtime.seconds() - danceTime < 4){
                robot.lock.setPosition(0.5);
                lockPosition = 0;
                robot.dumpHand.setPosition(0);
                dumpPosition = 0;
                robot.rightHand.setPosition(0);
                robot.leftHand.setPosition(0);
                spinDirection = 2;
                moveVertical(0);
                liftPosition = 0;
            }else{
                danceTime = runtime.seconds();
            }
        }
    }

}