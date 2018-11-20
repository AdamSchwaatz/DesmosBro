package org.firstinspires.ftc.teamcode.Brobotix;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.util.LynxOptimizedI2cFactory;
import org.jetbrains.annotations.NotNull;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;
import org.openftc.revextensions2.RevExtensions2;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * Optimized mecanum drive implementation for REV ExHs. The time savings here are enough to cut loop
 * iteration times in half which may significantly improve trajectory following performance.
 */
public class MecanumDriveBase extends SampleMecanumDriveBase {
    private ExpansionHubEx hub;
    public ExpansionHubMotor leftFrontMotor, leftRearMotor, rightRearMotor, rightFrontMotor, lift, handMotor, push;
    public Servo rightHand, leftHand, dumpHand, lock;
    private List<ExpansionHubMotor> motors;
    private BNO055IMU imu;
    public AnalogInput potentiometer = null;
    public WebcamName webcamName = null;

    public MecanumDriveBase(HardwareMap hardwareMap) {
        super();

        RevExtensions2.init();

        // TODO: adjust the names of the following hardware devices to match your configuration
        // for simplicity, we assume that the desired IMU and drive motors are on the same hub
        // note: this strategy is still applicable even if the drive motors are split between hubs
        hub = hardwareMap.get(ExpansionHubEx.class, "hub");

        imu = LynxOptimizedI2cFactory.createLynxEmbeddedImu(hub.getStandardModule(), 0);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        leftFrontMotor = hardwareMap.get(ExpansionHubMotor.class, "left_front");
        leftRearMotor = hardwareMap.get(ExpansionHubMotor.class, "left_rear");
        rightRearMotor = hardwareMap.get(ExpansionHubMotor.class, "right_rear");
        rightFrontMotor = hardwareMap.get(ExpansionHubMotor.class, "right_front");
        lift = hardwareMap.get(ExpansionHubMotor.class, "back_lift");
        handMotor = hardwareMap.get(ExpansionHubMotor.class, "hand_motor");
        push = hardwareMap.get(ExpansionHubMotor.class, "back_push");
        leftHand = hardwareMap.servo.get("left_hand");
        rightHand = hardwareMap.servo.get("right_hand");
        dumpHand = hardwareMap.servo.get("dump_hand");
        lock = hardwareMap.servo.get("lock");
        potentiometer = hardwareMap.analogInput.get("potentiometer");
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        motors = Arrays.asList(leftFrontMotor, leftRearMotor, rightRearMotor, rightFrontMotor);

        for (ExpansionHubMotor motor : motors) {
            // TODO: decide whether or not to use the built-in velocity PID
            // if you keep it, then don't tune kStatic or kA
            // otherwise, comment out the following line
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        handMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        push.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        handMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        push.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // TODO: reverse any motors using DcMotor.setDirection()
        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // TODO: set the tuned coefficients from DriveVelocityPIDTuner if using RUN_USING_ENCODER
         //setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDCoefficients(1,1,1));
    }

    @Override
    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode) {
        PIDFCoefficients coefficients = leftFrontMotor.getPIDFCoefficients(runMode);
        return new PIDCoefficients(coefficients.p, coefficients.i, coefficients.d);
    }

    @Override
    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients) {
        for (ExpansionHubMotor motor : motors) {
            motor.setPIDFCoefficients(runMode, new PIDFCoefficients(
                    coefficients.kP, coefficients.kI, coefficients.kD, 1
            ));
        }
    }

    public void teleop(){
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        handMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        push.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        push.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        handMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @NotNull
    @Override
    public List<Double> getWheelPositions() {
        RevBulkData bulkData = hub.getBulkInputData();
        List<Double> wheelPositions = new ArrayList<>();
        for (ExpansionHubMotor motor : motors) {
            wheelPositions.add(DriveConstants.encoderTicksToInches(bulkData.getMotorCurrentPosition(motor)));
        }
        return wheelPositions;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFrontMotor.setPower(v);
        leftRearMotor.setPower(v1);
        rightRearMotor.setPower(v2);
        rightFrontMotor.setPower(v3);
    }

    @Override
    public double getExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }
}
