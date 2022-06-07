package util.drive;

import util.misc.DreadbotMotor;

import com.kauailabs.navx.frc.AHRS;

public abstract class Drive {
    //Motor objects
    private DreadbotMotor frontLeftMotor;
    private DreadbotMotor frontRightMotor;
    private DreadbotMotor backLeftMotor;
    private DreadbotMotor backRightMotor;

    //NavX GyroScope
    private AHRS gyroscope;

    public Drive(DreadbotMotor frontLeftMotor, DreadbotMotor frontRightMotor,
                 DreadbotMotor backLeftMotor, DreadbotMotor backRightMotor, AHRS gyroscope) {
        this.frontLeftMotor = frontLeftMotor;
        this.frontRightMotor = frontRightMotor;
        this.backLeftMotor = backLeftMotor;
        this.backRightMotor = backRightMotor;

        this.gyroscope = gyroscope;
    }

    public AHRS getGyroscope() {
        return gyroscope;
    }
}
