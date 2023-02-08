package frc.robot.subsystems.drive.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;

public abstract class GyroIOWPI implements GyroIO {
    private Rotation2d m_offset;

    public GyroIOWPI() {
        m_offset = Rotation2d.fromDegrees(0);
    }

    @Override
    public abstract Gyro getWPIGyro();

    @Override
    public Rotation2d getRawGyroAngle() {
        return getWPIGyro().getRotation2d();
    }

    @Override
    public Rotation2d getOffset() {
        return m_offset;
    }

    @Override
    public double getRate() {
        return getWPIGyro().getRate();
    }

    @Override
    public void reset() {
        reset(Rotation2d.fromDegrees(0));
    }

    @Override
    public void reset(Rotation2d offset) {
        getWPIGyro().reset();
        m_offset = offset;
    }

    @Override
    public void addAngle(Rotation2d angle) {
        m_offset = m_offset.plus(angle);
    }
}
