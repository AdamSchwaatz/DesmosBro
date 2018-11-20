package org.firstinspires.ftc.teamcode.Brobotix;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 */
public class MecanumBasebot
{
    /* Public OpMode members. */
    public DcMotor leftFrontMotor   = null;
    public DcMotor rightFrontMotor  = null;
    public DcMotor leftRearMotor   = null;
    public DcMotor rightRearMotor  = null;
    public DcMotor lift = null;
    public DcMotor handMotor = null;
    public DcMotor push = null;
    public Servo rightHand = null;
    public Servo leftHand = null;
    public Servo dumpHand = null;
    public Servo lock = null;

    public AnalogInput potentiometer = null;

    public WebcamName webcamName = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public MecanumBasebot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFrontMotor   = hwMap.dcMotor.get("left_front");
        rightFrontMotor  = hwMap.dcMotor.get("right_front");
        leftRearMotor   = hwMap.dcMotor.get("left_rear");
        rightRearMotor  = hwMap.dcMotor.get("right_rear");
        lift = hwMap.dcMotor.get("back_lift");
        handMotor = hwMap.dcMotor.get("hand_motor");
        push = hwMap.dcMotor.get("back_push");

        potentiometer = hwMap.analogInput.get("potentiometer");

        webcamName = hwMap.get(WebcamName.class, "Webcam 1");

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        handMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        push.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRearMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        handMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        push.setDirection(DcMotorSimple.Direction.FORWARD);

        leftHand = hwMap.servo.get("left_hand");
        rightHand = hwMap.servo.get("right_hand");
        dumpHand = hwMap.servo.get("dump_hand");
        lock = hwMap.servo.get("lock");

        // Set all motors to zero power
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
        rightRearMotor.setPower(0);
        lift.setPower(0);
        handMotor.setPower(0);
        push.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        handMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        push.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }


}

