package org.firstinspires.ftc.teamcode.Brobotix;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.Dogeforia;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.Locale;

/**
 * Created by Adam on 11/27/2017.
 */
//@Disabled
@Autonomous(name="CraterSide", group ="Concept")

public class CraterSide extends LinearOpMode {

    //0:Straight to depot
    //1:Around minerals to the other side of the depot
    static final int selection1 = 0;
    //0:To our side
    //1:To opposite team crater
    static final int selection2 = 0;
    //0:Straight to left side of crater
    //1:Around minerals to right side of crater
    //**Note: They are opposite side of crater if the other alliance crater**
    static final int selection3 = 0;

    public enum GoldLocation {
        UNKNOWN,
        LEFT,
        CENTER,
        RIGHT
    }

    GoldLocation location = GoldLocation.UNKNOWN;

    Dogeforia vuforia;
    GoldAlignDetector detector;
    /* Declare OpMode members. */
    // Use a Pushbot's hardware
    MecanumBasebot robot   = new MecanumBasebot();
    private ElapsedTime runtime = new ElapsedTime();

    static final double     FORWARD_SPEED =  1;
    static final double     TURN_SPEED    = 1;
    static final double     ticks = 1120*0.89;
    static final double     ticksRevHD = 2240;
    static final double     ticksCoreHex = 288;
    static final int        WHEEL_DIAMETER_INCHES = 4;
    static final int        SPOOL_DIAMETER_INCHES = 1;
    static final double     SHAFT_DIAMETER_INCHES = 0.75;
    static final int        DRIVE_GEAR_REDUCTION = 1;
    static final double     VERTICAL_GEAR_REDUCTION = 6;
    static final int        HORIZONTAL_GEAR_REDUCTION = 1;
    static final int        HAND_GEAR_REDUCTION = 1;
    static final double     PI  = 3.1415;
    static final double     DISTANCE_FOR_90 = (7 * PI / 2) + 1.5;
    static final double COUNTS_PER_INCH =(ticks * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * PI);
    static final double COUNTS_PER_INCH_HORIZONTAL =(ticksRevHD * HORIZONTAL_GEAR_REDUCTION) / (SPOOL_DIAMETER_INCHES * PI);
    static final double COUNTS_PER_INCH_VERTICAL =(ticksCoreHex * VERTICAL_GEAR_REDUCTION) / (SPOOL_DIAMETER_INCHES * PI);
    static final double COUNTS_PER_INCH_HAND =(ticksCoreHex * HAND_GEAR_REDUCTION) / (SHAFT_DIAMETER_INCHES * PI);
    int depotSelection;
    int craterSelection;

    //Setup for the internal imu in the expansion hub
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    @Override public void runOpMode(){
        //Initialize the HarwareBasebot
        robot.init(hardwareMap);

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = "AddwF3b/////AAABmZBtbnIyn0YtiY+mFOvH/xEd0Grx26TxJIQS5eDfhbpz0pnAfpIK/nADPReSRsxR2OGZc8E/cQGdgVNuROqPVK2m0pKeg0r+7t8uykTbIdjkb9YT0rZP5k1vWUwC5AOZakLsCBYYBbsikyIeYR9i4PSk31VHgHGLz9C/c/W1PjgBxNVbKc3V2nyi69MfMaI8LI0dYgnlmhaoI9cYRSzOqAJkLog/W/h+X0jUDLxW+fpSfDbmdhpV33/cwUAKqLlwr4FEo54hyaJMkphfWj1HBeioEGLxUkvRuqsTl7jP7Fdz9S9O7y548hnf0eCx7YzOyecrmcRTOvGEvDWlaJmTnDCVjZvMhGy5mchU2tx+9KYP";
        parameters.fillCameraMonitorViewParent = true;
        parameters.cameraName = robot.webcamName;
        vuforia = new Dogeforia(parameters);
        vuforia.enableConvertFrameToBitmap();

        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance(),0,true);
        detector.useDefaults();

        // Optional Tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.PERFECT_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        vuforia.setDogeCVDetector(detector);
        vuforia.enableDogeCV();
        vuforia.showDebug();
        vuforia.start();

        //Instantiate the imu and all of it's parameters
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters2 = new BNO055IMU.Parameters();
        parameters2.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters2.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters2.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters2.loggingEnabled      = true;
        parameters2.loggingTag          = "IMU";
        parameters2.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters2);
        //Wait for start and show imu data
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        composeTelemetry();

        //robot.lift.setPower(0.5);
        waitForStart();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        telemetry.addData("Front Left ", String.format("number: " + robot.leftFrontMotor.getCurrentPosition()));
        telemetry.addData("Front Right ", String.format("number: " + robot.rightFrontMotor.getCurrentPosition()));
        telemetry.addData("Back Left ", String.format("number: " + robot.leftRearMotor.getCurrentPosition()));
        telemetry.addData("Back Right ", String.format("number: " + robot.rightRearMotor.getCurrentPosition()));
        //Run the autonomous program
        run();
        vuforia.stop();


    }

    void run(){
        deployRobot();
        test();
        alignAndKnock();
        toSpot();
        //0:Straight to depot
        //1:Around minerals to the other side of the depot
        toDepot(selection1);
        dropGamePiece();
        //0:To our side
        //1:To opposite team crater
        backToSpot(selection2);
        //0:Straight to left side of crater
        //1:Around minerals to right side of crater
        toCrater(selection3);
        motorsStop();
    }

    void deployRobot(){
        robot.lock.setPosition(0);
        moveVertical(9,1,0);
        face0();
    }
    void test(){
        faceToTheLeft(32-5);
        if(detector.isFound()){
            double x = detector.getXPosition();
            if(x>100&&x<550){
                location = GoldLocation.LEFT;
            }
        }
        if(location == GoldLocation.UNKNOWN){
            faceToTheRight(27);
            if(detector.isFound()){
                double x = detector.getXPosition();
                if(x>100&&x<550) {
                    location = GoldLocation.RIGHT;
                }
            }
        }
        if(location == GoldLocation.UNKNOWN) {
            face0();
            if (detector.isFound()) {
                location = GoldLocation.CENTER;
            }
        }
        detector.disable();
        vuforia.stop();
    }
    void alignAndKnock(){
        switch(location){
            case UNKNOWN:
                face0();
                move(25+9);
                break;
            case LEFT:
                faceToTheLeft(32);
                move(29+9);
                break;
            case CENTER:
                move(25+9);
                break;
            case RIGHT:
                faceToTheRight(32);
                move(29+9);
                break;
            default:
                move(25+9);
                break;
        }
    }
    void toSpot(){
        switch(location){
            case UNKNOWN:
                moveBackward(25+8);
                faceToTheLeft(45);
                move(52);
                break;
            case LEFT:
                moveBackward(29+8);
                faceToTheLeft(45);
                move(52);
                break;
            case CENTER:
                moveBackward(25+8);
                faceToTheLeft(45);
                move(52);
                break;
            case RIGHT:
                moveBackward(29+8);
                faceToTheLeft(45);
                move(52);
                break;
            default:
                moveBackward(25+8);
                faceToTheLeft(45);
                move(52);
        }
    }
    void toDepot(int selection){
        if(selection == 0){
            depotSelection = 0;
            faceToTheLeft(135-5);
            move(72-9);
        }else if(selection == 1){
            depotSelection = 1;
            faceToTheLeft(5);
//            face0();
            moveBackward(80);
            faceToTheLeft(45);
            move(48);
        }
    }
    void dropGamePiece(){
        robot.leftHand.setPosition(0);
        robot.rightHand.setPosition(0);
        sleep2(2);
        robot.rightHand.setPosition(0.5);
        robot.leftHand.setPosition(0.5);
    }
    void backToSpot(int selection){
        if(selection == 0 ) {
            craterSelection = 0;
            if (depotSelection == 0) {
                faceToTheLeft(135);
                moveBackward(60);
            } else if (depotSelection == 1) {
                moveBackward(45);
                face0();
                move(80);
                faceToTheRight(45 + 2);
            }
        }else if(selection == 1){
            craterSelection = 1;
            faceToTheLeft(45);
            moveBackward(60);
        }
    }
    void toCrater(int selection){
        if(craterSelection == 0) {
            if (selection == 0) {
                moveBackward(30);
            } else if (selection == 1) {
                faceToTheRight(90);
                move(80);
                face0();
                move(24);
            }
        }else if(craterSelection == 1){
            if(selection == 0){
                moveBackward(30);
            }else if(selection == 1){
                faceToTheLeft(90);
                moveBackward(80);
                face0();
                moveBackward(24);
            }
        }
    }

    void moveVertical( double inches,double speed, int direction){
        //Intended to move the distance forward in inches passed to the function
        // How far are we to move, in ticks instead of revolutions
        int denc = (int)Math.round(inches * COUNTS_PER_INCH_VERTICAL);
        // Tell the motors where we are going
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int liftDistance;
        if(direction == 0){
            liftDistance = robot.lift.getCurrentPosition() - (denc);
        }else if(direction == 1){
            liftDistance = robot.lift.getCurrentPosition() + (denc);
        }else{
            liftDistance = 0;
        }
        robot.lift.setTargetPosition(liftDistance);
        //Run
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Give them the power level
        robot.lift.setPower(speed);
        //Wait until they are done
        runtime.reset();
        while(robot.lift.isBusy()&&runtime.seconds()<5){
            robot.lift.setPower(speed);
        }
        //Stop the motors
        robot.lift.setPower(0);
    }
    void moveHorizontal( double inches,double speed, int direction){
        //Intended to move the distance forward in inches passed to the function
        // How far are we to move, in ticks instead of revolutions
        int denc = (int)Math.round(inches * COUNTS_PER_INCH_HORIZONTAL);
        // Tell the motors where we are going
        robot.push.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int pushDistance;
        if(direction == 0){
            pushDistance = robot.push.getCurrentPosition() + (denc);
        }else if(direction == 1){
            pushDistance = robot.push.getCurrentPosition() - (denc);
        }else{
            pushDistance = 0;
        }
        robot.push.setTargetPosition(pushDistance);
        //Run
        robot.push.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Give them the power level
        robot.push.setPower(speed);
        //Wait until they are done
        while(robot.push.isBusy()){
            robot.push.setPower(speed);
        }
        //Stop the motors
        robot.push.setPower(0);
    }
    void moveHand( double inches,double speed, int direction){

        //Intended to move the distance forward in inches passed to the function
        // How far are we to move, in ticks instead of revolutions
        int denc = (int)Math.round(inches * COUNTS_PER_INCH_HORIZONTAL);
        // Tell the motors where we are going
        robot.handMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int spinDistance;
        if(direction == 0){
            spinDistance = robot.handMotor.getCurrentPosition() - (denc);
        }else if(direction == 1){
            spinDistance = robot.handMotor.getCurrentPosition() + (denc);
        }else{
            spinDistance = 0;
        }
        robot.handMotor.setTargetPosition(spinDistance);
        //Run
        robot.handMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Give them the power level
        robot.handMotor.setPower(speed);
        //Wait until they are done
        while(robot.handMotor.isBusy()){
            robot.handMotor.setPower(speed);
        }
        //Stop the motors
        robot.handMotor.setPower(0);
    }
    void motorsForward(){
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftFrontMotor.setPower(FORWARD_SPEED);
        robot.rightFrontMotor.setPower(FORWARD_SPEED);
        robot.leftRearMotor.setPower(FORWARD_SPEED);
        robot.rightRearMotor.setPower(FORWARD_SPEED);
    }
    void motorsStop(){
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftFrontMotor.setPower(0);
        robot.rightFrontMotor.setPower(0);
        robot.leftRearMotor.setPower(0);
        robot.rightRearMotor.setPower(0);
    }
    void moveMotors(double leftF, double rightF, double leftR, double rightR, double inches,double speed){
        //Intended to move the distance forward in inches passed to the function
        // How far are we to move, in ticks instead of revolutions
        int denc = (int)Math.round(inches * COUNTS_PER_INCH);
        // Tell the motors where we are going
        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int leftFront = robot.leftFrontMotor.getCurrentPosition() + (int)(denc*leftF);
        int rightFront = robot.rightFrontMotor.getCurrentPosition() + (int)(denc*rightF);
        int leftRear = robot.leftRearMotor.getCurrentPosition() + (int)(denc*leftR);
        int rightRear = robot.rightRearMotor.getCurrentPosition() + (int)(denc*rightR);
        robot.leftFrontMotor.setTargetPosition(leftFront);
        robot.rightFrontMotor.setTargetPosition(rightFront);
        robot.leftRearMotor.setTargetPosition(leftRear);
        robot.rightRearMotor.setTargetPosition(rightRear);
        //Run
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Give them the power level
        robot.leftFrontMotor.setPower(speed);
        robot.rightFrontMotor.setPower(speed);
        robot.leftRearMotor.setPower(speed);
        robot.rightRearMotor.setPower(speed);
        //Wait until they are done
        double startTime = runtime.seconds();
        while((robot.leftFrontMotor.isBusy() ||robot.rightFrontMotor.isBusy() || robot.leftRearMotor.isBusy() || robot.rightRearMotor.isBusy())&&(runtime.seconds()-startTime)<5){
            robot.leftFrontMotor.setPower(speed);
            robot.rightFrontMotor.setPower(speed);
            robot.leftRearMotor.setPower(speed);
            robot.rightRearMotor.setPower(speed);
        }
        //Stop the motors
        robot.leftFrontMotor.setPower(0);
        robot.rightFrontMotor.setPower(0);
        robot.leftRearMotor.setPower(0);
        robot.rightRearMotor.setPower(0);
    }
    void moveMotorsFast(double leftF, double rightF, double leftR, double rightR, double inches,double speed){
        //Intended to move the distance forward in inches passed to the function
        // How far are we to move, in ticks instead of revolutions
        int denc = (int)Math.round(inches * COUNTS_PER_INCH);
        // Tell the motors where we are going
        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int leftFront = robot.leftFrontMotor.getCurrentPosition() + (int)(denc*leftF);
        int rightFront = robot.rightFrontMotor.getCurrentPosition() + (int)(denc*rightF);
        int leftRear = robot.leftRearMotor.getCurrentPosition() + (int)(denc*leftR);
        int rightRear = robot.rightRearMotor.getCurrentPosition() + (int)(denc*rightR);
        robot.leftFrontMotor.setTargetPosition(leftFront);
        robot.rightFrontMotor.setTargetPosition(rightFront);
        robot.leftRearMotor.setTargetPosition(leftRear);
        robot.rightRearMotor.setTargetPosition(rightRear);
        //Run
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Give them the power level
        robot.leftFrontMotor.setPower(speed);
        robot.rightFrontMotor.setPower(speed);
        robot.leftRearMotor.setPower(speed);
        robot.rightRearMotor.setPower(speed);
        //Wait until they are done
        double startTime = runtime.seconds();
        while((robot.leftFrontMotor.isBusy() ||robot.rightFrontMotor.isBusy() || robot.leftRearMotor.isBusy() || robot.rightRearMotor.isBusy())&&(runtime.seconds()-startTime)<1){
            robot.leftFrontMotor.setPower(speed);
            robot.rightFrontMotor.setPower(speed);
            robot.leftRearMotor.setPower(speed);
            robot.rightRearMotor.setPower(speed);
        }
        //Stop the motors
        robot.leftFrontMotor.setPower(0);
        robot.rightFrontMotor.setPower(0);
        robot.leftRearMotor.setPower(0);
        robot.rightRearMotor.setPower(0);
    }
    void move(double inches){
        moveMotors(1,1,1,1,inches,FORWARD_SPEED);
    }
    void moveBackward(double inches){
        moveMotors(-1,-1,-1,-1,inches, FORWARD_SPEED);
    }
    void turnRight(double inches){
        moveMotorsFast(1,-1,1,-1,inches, TURN_SPEED);
    }
    void faceLeft(){
        //Face 90 degrees left of the starting direction
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runtime.reset();
        while(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle<82.5 && runtime.seconds() < 1) {
            robot.leftFrontMotor.setPower(-TURN_SPEED);
            robot.rightFrontMotor.setPower(TURN_SPEED);
            robot.leftRearMotor.setPower(-TURN_SPEED);
            robot.rightRearMotor.setPower(TURN_SPEED);
        }
        robot.rightFrontMotor.setPower(0);
        robot.leftFrontMotor.setPower(0);
        robot.rightRearMotor.setPower(0);
        robot.leftRearMotor.setPower(0);
        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    void faceRight(){
        //Face 90 degrees right of the starting direction
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runtime.reset();
        while(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle>-82.5 && runtime.seconds() < 1) {
            robot.leftFrontMotor.setPower(TURN_SPEED);
            robot.rightFrontMotor.setPower(-TURN_SPEED);
            robot.leftRearMotor.setPower(TURN_SPEED);
            robot.rightRearMotor.setPower(-TURN_SPEED);
        }
        if(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle>-82.5 && runtime.seconds() < 1){
            while(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle>-82.5) {
                robot.leftFrontMotor.setPower(TURN_SPEED+0.1);
                robot.rightFrontMotor.setPower(-TURN_SPEED+0.1);
                robot.leftRearMotor.setPower(TURN_SPEED+0.1);
                robot.rightRearMotor.setPower(-TURN_SPEED+0.1);
            }
        }
        robot.rightFrontMotor.setPower(0);
        robot.leftFrontMotor.setPower(0);
        robot.rightRearMotor.setPower(0);
        robot.leftRearMotor.setPower(0);
        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    void face0(){
        //Return the to facing the original direction the started
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot. rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        float angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        runtime.reset();
        if(angle > 0) {
            while (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle > 7.5&& runtime.seconds() < 1) {
                robot.leftFrontMotor.setPower(TURN_SPEED);
                robot.rightFrontMotor.setPower(-TURN_SPEED);
                robot.leftRearMotor.setPower(TURN_SPEED);
                robot.rightRearMotor.setPower(-TURN_SPEED);
            }
        }else if(angle < 0){
            while (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle < -7.5 && runtime.seconds() < 1) {
                robot.leftFrontMotor.setPower(-TURN_SPEED);
                robot.rightFrontMotor.setPower(TURN_SPEED);
                robot.leftRearMotor.setPower(-TURN_SPEED);
                robot.rightRearMotor.setPower(TURN_SPEED);
            }
        }
        robot.rightFrontMotor.setPower(0);
        robot.leftFrontMotor.setPower(0);
        robot.rightRearMotor.setPower(0);
        robot.leftRearMotor.setPower(0);
        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    void faceBackward(){
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runtime.reset();
        while(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle<172.5 && runtime.seconds() < 1) {
            robot.leftFrontMotor.setPower(TURN_SPEED);
            robot.rightFrontMotor.setPower(-TURN_SPEED);
            robot.leftRearMotor.setPower(TURN_SPEED);
            robot.rightRearMotor.setPower(-TURN_SPEED);
        }
        robot.rightFrontMotor.setPower(0);
        robot.leftFrontMotor.setPower(0);
        robot.rightRearMotor.setPower(0);
        robot.leftRearMotor.setPower(0);
        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    void faceToTheRight(int degrees){
        //Face 90 degrees right of the starting direction
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runtime.reset();
        while(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle>((-degrees)+7.5) && runtime.seconds() < 2) {
            robot.leftFrontMotor.setPower(TURN_SPEED);
            robot.rightFrontMotor.setPower(-TURN_SPEED);
            robot.leftRearMotor.setPower(TURN_SPEED);
            robot.rightRearMotor.setPower(-TURN_SPEED);
        }
        if(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle>((-degrees)+7.5) && runtime.seconds() < 2){
            while(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle>((-degrees)+7.5)) {
                robot.leftFrontMotor.setPower(TURN_SPEED+0.1);
                robot.rightFrontMotor.setPower(-TURN_SPEED+0.1);
                robot.leftRearMotor.setPower(TURN_SPEED+0.1);
                robot.rightRearMotor.setPower(-TURN_SPEED+0.1);
            }
        }
        robot.rightFrontMotor.setPower(0);
        robot.leftFrontMotor.setPower(0);
        robot.rightRearMotor.setPower(0);
        robot.leftRearMotor.setPower(0);
        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    void faceToTheLeft(int degrees){
        //Face 90 degrees left of the starting direction
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runtime.reset();
        while(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle<(degrees-7.5) && runtime.seconds() < 2) {
            robot.leftFrontMotor.setPower(-TURN_SPEED);
            robot.rightFrontMotor.setPower(TURN_SPEED);
            robot.leftRearMotor.setPower(-TURN_SPEED);
            robot.rightRearMotor.setPower(TURN_SPEED);
        }
        robot.rightFrontMotor.setPower(0);
        robot.leftFrontMotor.setPower(0);
        robot.rightRearMotor.setPower(0);
        robot.leftRearMotor.setPower(0);
        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    void turnLeft(double inches){
        moveMotorsFast(-1,1,-1,1,inches,TURN_SPEED);
    }
    void strafeLeft(double inches){
        moveMotors(-1,1,1,-1,inches*2,TURN_SPEED);
    }
    void strafeRight(double inches){
        moveMotors(1,-1,-1,1,inches*2,TURN_SPEED);
    }
    void sleep2(double seconds){
        double startTime = runtime.seconds();
        while ((runtime.seconds()-startTime) < seconds) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }
    void composeTelemetry() {
        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });
        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });
        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });
        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}