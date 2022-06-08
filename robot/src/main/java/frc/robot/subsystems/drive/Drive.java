package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.DreadbotSubsystem;
import util.DreadbotMotor;
import util.math.DreadbotMath;

import java.util.stream.Stream;

/**
 * The drive is the mechanism that moves the robot across the field. We are using a mecanum drive.
 */
public class Drive extends DreadbotSubsystem {
    // Motor Objects
    protected DreadbotMotor leftFrontMotor;
    protected DreadbotMotor rightFrontMotor;
    protected DreadbotMotor leftBackMotor;
    protected DreadbotMotor rightBackMotor;

    // NavX Gyroscope
    protected AHRS gyroscope;

    // Target ChassisSpeeds commanded by teleop directions or
    protected ChassisSpeeds targetChassisSpeeds;

    @Override
    public void stopMotors() {
        if(isDisabled()) return;

        try {
            //mecanumDrive.stopMotor();
        } catch (IllegalStateException ignored) { disable(); }
    }

    @Override
    public void close() {
        // Stop motors before closure
        stopMotors();

        try {
            leftFrontMotor.close();
            rightFrontMotor.close();
            leftBackMotor.close();
            rightBackMotor.close();
        } catch (IllegalStateException ignored) { disable(); }
    }

    /**
     * Resets the encoder positions.
     */
    public void resetEncoders() {
        if(isDisabled()) return;

        rightFrontMotor.resetEncoder();
        leftFrontMotor.resetEncoder();
    }

    /**
     * Returns the gyroscope yaw.
     *
     * @return The gyroscope yaw
     */
    public double getYaw() {
        return gyroscope.getYaw();
    }

    public AHRS getGyroscope() {
        return gyroscope;
    }
}