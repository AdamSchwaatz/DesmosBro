package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.hardware.motors.NeveRest20Gearmotor;
import com.qualcomm.hardware.motors.NeveRest40Gearmotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

/*
 * Constants shared between multiple drive types.
 */
@Config
public class DriveConstants {

    private DriveConstants() {

    }

    /*
     * TODO: Tune or adjust the following constants to fit your robot. Note that the non-final
     * fields may also be edited through the dashboard (connect to the robot's WiFi network and
     * navigate to https://192.168.49.1:8080/dash). Make sure to save the values here after you
     * adjust them in the dashboard; **config variable changes don't persist between app restarts**.
     */
    public static final MotorConfigurationType MOTOR_CONFIG =
            MotorConfigurationType.getMotorType(NeveRest40Gearmotor.class);
    private static final double TICKS_PER_REV = MOTOR_CONFIG.getTicksPerRev();
//private static final double TICKS_PER_REV = 1120*0.89;
    public static double WHEEL_RADIUS = 4; // in
    public static double GEAR_RATIO = 1; // output/input
    public static double TRACK_WIDTH = 14.66; // in

    public static DriveConstraints BASE_CONSTRAINTS = new DriveConstraints(50.0, 50.0, 3, 3);

    public static double kV = 0.01725;
    public static double kA = 0.00089;
    public static double kStatic = 0.01797;


    public static double encoderTicksToInches(int ticks) {

        return WHEEL_RADIUS  * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {

        return rpm * DriveConstants.GEAR_RATIO  * Math.PI * DriveConstants.WHEEL_RADIUS / 60.0 ;
    }
}
