// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.commands.drive;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** 
 * An abstract command which provides base logic for driving a swerve base.
 * Inherit from this class to provide functionality on how the swerve drive should move
 * in the x, y, and rotational axes, and whether it should move relative to the robot (robot relative)
 * or relative to the driver (field relative).
 * This command class is highly adapted from 2363's swerve drive commands
 */
public abstract class BaseDriveCommand extends CommandBase {
  protected final DriveCommandConfig m_drive;
  // PIDController headingPIDController = new PIDController(DriveConstants.kHeadingP, DriveConstants.kHeadingI,
  //     DriveConstants.kHeadingD);

  public BaseDriveCommand(DriveCommandConfig drive) {
    m_drive = requireNonNullParam(drive, "drive", "BaseDriveCommand");
    addRequirements(drive.getRequirements());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public final void execute() {
    double x = getXSpeed();
    double y = getYSpeed();
    double omega = getRotationSpeed();
    boolean isFieldRelative = getFieldRelative();

    Pose2d robotPoseVel = new Pose2d(x * 0.02, y * 0.02, Rotation2d.fromRadians(omega * 0.02));
    Twist2d twistVel = new Pose2d(0, 0, Rotation2d.fromRadians(0)).log(robotPoseVel);
    ChassisSpeeds newChassisSpeeds = getChassisSpeeds(
        twistVel.dx / 0.02, twistVel.dy / 0.02, twistVel.dtheta / 0.02, isFieldRelative);

    m_drive.acceptSpeeds(newChassisSpeeds);
    // System.out.println("i'm running!");
  }

  /**
  * Combines all returned movement values to create output chassis ChassisSpeeds
  * Putting this logic in its own method allows {@link DriveCommandWrapper}s to
  * modify the late stage chassis speeds being sent to the drive command
  * Do <strong>NOT</strong> override this method unless you know what you are doing!
  * @return the desired robot chassis speeds
  */
  protected ChassisSpeeds getChassisSpeeds(double x, double y, double omega, boolean isFieldRelative) {
    return isFieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(x, y, omega, m_drive.getPose().getRotation())
        : new ChassisSpeeds(x, y, omega);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.acceptSpeeds(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public final boolean runsWhenDisabled() {
    return false;
  }

  /**
   * @return the x (forward positive) speed in meters per second
   */
  protected abstract double getXSpeed();

  /**
   * @return the y (left positive) speed in meters per second 
   */
  protected abstract double getYSpeed();

  /**
   * @return the angular (CCW positive) speed in radians per second
   */
  protected abstract double getRotationSpeed();

  /**
   * Determines whether robot speeds are relative to the driver (field relative)
   * or relative to the robot (robot relative)
   * @return true if field relative, false if robot relative
   */
  protected abstract boolean getFieldRelative();

  /**
   * @param xSpeedSupplier provides the x speed input [-1, 1]
   * @param ySpeedSupplier provides the y speed input [-1, 1]
   * @return a wrapped drive command which uses the specified linear movement input suppliers
   */
  public final BaseDriveCommand withLinearSpeedSuppliers(Supplier<Double> xSpeedSupplier,
      Supplier<Double> ySpeedSupplier) {
    return new DriveCommandWrapper(this) {
      @Override
      protected double getXSpeed() {
        return xSpeedSupplier.get();
      }

      @Override
      protected double getYSpeed() {
        return ySpeedSupplier.get();
      }
    };
  }

  /**
  * @param xInputSupplier provides the x speed input [-1, 1]
  * @param yInputSupplier provides the y speed input [-1, 1]
  * @return a wrapped drive command which uses the specified angular movement input supplier
  */
  public final BaseDriveCommand withAngularSpeedSupplier(Supplier<Double> omegaSupplier) {
    return new DriveCommandWrapper(this) {
      @Override
      protected double getRotationSpeed() {
        return omegaSupplier.get();
      }
    };
  }

  /**
   * Returns a new wrapped {@link BaseDriveCommand} with field relative set to the desired value
   * @param enabled
   * @return
   */
  public final BaseDriveCommand withFieldRelative(boolean enabled) {
    return new DriveCommandWrapper(this) {
      @Override
      protected boolean getFieldRelative() {
        return enabled;
      }
    };
  }

  /**
   * Creates a {@link BaseDriveCommand} builder class with the given config.
   * This is the most efficient way to create a custom drive command without introducing numerous wrapper classes.
   * @param config
   * @return
   */
  public static DriveCommandBuilder builder(DriveCommandConfig config) {
    return new DriveCommandBuilder(config);
  }

  public static class DriveCommandBuilder {
    private static final boolean kDefaultUsePercentSpeeds = false;
    private static final boolean kDefaultUseFieldRelative = false;
    private static final Supplier<Double> kDefaultDoubleSupplier = () -> 0.0;
    private static final double kDefaultTranslationalSpeedConstraint = Units.feetToMeters(3);
    private static final double kDefaultAngularSpeedConstraint = Units.degreesToRadians(180);

    private final DriveCommandConfig m_config;
    private Optional<Supplier<Translation2d>> m_faceTarget = Optional.empty();
    private Optional<Supplier<Rotation2d>> m_targetAngleSupplier = Optional.empty();
    private Optional<Supplier<Optional<Rotation2d>>> m_safeTargetAngleSupplier = Optional.empty();
    private Optional<ProfiledPIDController> m_turnPIDController = Optional.empty();
    private Optional<Supplier<Double>> m_xSpeedSupplier = Optional.empty();
    private Optional<Supplier<Double>> m_ySpeedSupplier = Optional.empty();
    private Optional<Supplier<Double>> m_angularSpeedSupplier = Optional.empty();
    private Optional<Supplier<Boolean>> m_useFieldRelative = Optional.empty();
    private Optional<Double> m_translationalSpeedConstraint = Optional.empty();
    private Optional<Double> m_angularSpeedConstraint = Optional.empty();
    private Optional<Boolean> m_usePercentSpeeds = Optional.empty();

    private DriveCommandBuilder(DriveCommandConfig config) {
      m_config = requireNonNullParam(config, "config", "DriveCommandBuilder");
    }

    public BaseDriveCommand build() {
      var xSpeedSupplier = getXSpeedSupplier();
      var ySpeedSupplier = getYSpeedSupplier();
      var angularSpeedSupplier = getAngularSpeedSupplier();
      var fieldRelativeSupplier = getFieldRelativeSupplier();
      double translationalSpeedConstraint = getTranslationalSpeedConstraint();
      double angularSpeedConstraint = getAngularSpeedConstraint();
      boolean usePercent = usePercentSpeeds();

      /*
       * If the user provided a target to face towards, return a FaceTargetDrive command
       */
      if (m_faceTarget.isPresent()) {
        ProfiledPIDController controller = m_turnPIDController.orElseThrow();
        return new FaceTargetDrive(m_config, controller, m_faceTarget.get()) {
          @Override
          protected double getXSpeed() {
            return getCalculatedSpeed(xSpeedSupplier.get(), usePercent, translationalSpeedConstraint);
          };

          @Override
          protected double getYSpeed() {
            return getCalculatedSpeed(ySpeedSupplier.get(), usePercent, translationalSpeedConstraint);
          };

          @Override
          protected boolean getFieldRelative() {
            return fieldRelativeSupplier.get();
          };
        };
      }

      /*
      * If the user provided a SAFE targetAngleSupplier, return a SafeTargetAngleDrive command
      */
      if (m_safeTargetAngleSupplier.isPresent()) {
        var safeTargetAngleSupplier = m_safeTargetAngleSupplier.get();
        ProfiledPIDController controller = m_turnPIDController.orElseThrow();
        return new SafeTargetAngleDrive(m_config, controller) {
          @Override
          protected double getXSpeed() {
            return getCalculatedSpeed(xSpeedSupplier.get(), usePercent, translationalSpeedConstraint);
          };

          @Override
          protected double getYSpeed() {
            return getCalculatedSpeed(ySpeedSupplier.get(), usePercent, translationalSpeedConstraint);
          };

          @Override
          protected boolean getFieldRelative() {
            return fieldRelativeSupplier.get();
          };

          @Override
          protected Optional<Rotation2d> getDesiredSafeAngle() {
            return safeTargetAngleSupplier.get();
          }
        };
      }

      /*
       * If the user provided a targetAngleSupplier, return a TargetAngleDrive command
       */
      if (m_targetAngleSupplier.isPresent()) {
        var targetAngleSupplier = m_targetAngleSupplier.get();
        ProfiledPIDController controller = m_turnPIDController.orElseThrow();
        return new TargetAngleDrive(m_config, controller) {
          @Override
          protected double getXSpeed() {
            return getCalculatedSpeed(xSpeedSupplier.get(), usePercent, translationalSpeedConstraint);
          };

          @Override
          protected double getYSpeed() {
            return getCalculatedSpeed(ySpeedSupplier.get(), usePercent, translationalSpeedConstraint);
          };

          @Override
          protected boolean getFieldRelative() {
            return fieldRelativeSupplier.get();
          };

          @Override
          protected Rotation2d getDesiredAngle() {
            return targetAngleSupplier.get();
          }
        };
      }

      /*
       * Default to a basic drive implementation with the provided inputs
       */
      return new DriveCommandAdapter(m_config) {
        protected double getXSpeed() {
          return getCalculatedSpeed(xSpeedSupplier.get(), usePercent, translationalSpeedConstraint);
        };

        protected double getYSpeed() {
          return getCalculatedSpeed(ySpeedSupplier.get(), usePercent, translationalSpeedConstraint);
        };

        protected double getRotationSpeed() {
          return getCalculatedSpeed(angularSpeedSupplier.get(), usePercent, angularSpeedConstraint);
        };

        protected boolean getFieldRelative() {
          return fieldRelativeSupplier.get();
        };
      };
    }

    public DriveCommandBuilder facesTarget(Supplier<Pose2d> pose, ProfiledPIDController controller) {
      m_faceTarget = Optional.ofNullable(() -> pose.get().getTranslation());
      m_turnPIDController = Optional.ofNullable(controller);
      return this;
    }

    public DriveCommandBuilder facesTarget(Pose2d pose, ProfiledPIDController controller) {
      return facesTarget(() -> pose, controller);
    }

    public DriveCommandBuilder facesTarget(Pose3d pose, ProfiledPIDController controller) {
      return facesTarget(() -> pose.toPose2d(), controller);
    }

    public DriveCommandBuilder withTargetAngleSupplier(Supplier<Rotation2d> targetAngle,
        ProfiledPIDController controller) {
      m_targetAngleSupplier = Optional.ofNullable(targetAngle);
      m_turnPIDController = Optional.ofNullable(controller);
      return this;
    }

    public DriveCommandBuilder withSafeTargetAngleSupplier(Supplier<Optional<Rotation2d>> targetAngle,
        ProfiledPIDController controller) {
      m_safeTargetAngleSupplier = Optional.ofNullable(targetAngle);
      m_turnPIDController = Optional.ofNullable(controller);
      return this;
    }

    public DriveCommandBuilder withSpeedSuppliers(
        Supplier<Double> xSpeedSupplier,
        Supplier<Double> ySpeedSupplier,
        Supplier<Double> rotationSpeedSupplier) {
      this.m_xSpeedSupplier = Optional.ofNullable(xSpeedSupplier);
      this.m_ySpeedSupplier = Optional.ofNullable(ySpeedSupplier);
      this.m_angularSpeedSupplier = Optional.ofNullable(rotationSpeedSupplier);
      return this;
    }

    public DriveCommandBuilder withLinearSpeedSuppliers(
        Supplier<Double> xSpeedSupplier,
        Supplier<Double> ySpeedSupplier) {
      this.m_xSpeedSupplier = Optional.ofNullable(xSpeedSupplier);
      this.m_ySpeedSupplier = Optional.ofNullable(ySpeedSupplier);
      return this;
    }

    public DriveCommandBuilder withAngularSpeedSupplier(
        Supplier<Double> angularSpeedSupplier) {
      this.m_angularSpeedSupplier = Optional.ofNullable(angularSpeedSupplier);
      return this;
    }

    public DriveCommandBuilder useFieldRelative(Supplier<Boolean> enabled) {
      this.m_useFieldRelative = Optional.ofNullable(enabled);
      return this;
    }

    public DriveCommandBuilder useFieldRelative(boolean enabled) {
      return useFieldRelative(() -> enabled);
    }

    public DriveCommandBuilder withTranslationalSpeedConstraint(double metersPerSecond) {
      this.m_translationalSpeedConstraint = Optional.of(metersPerSecond);
      return this;
    }

    public DriveCommandBuilder withAngularSpeedConstraint(double radiansPerSecond) {
      this.m_angularSpeedConstraint = Optional.of(radiansPerSecond);
      return this;
    }

    public DriveCommandBuilder withSpeedConstraints(double translationMetersPerSecond, double angularRadiansPerSecond) {
      return withTranslationalSpeedConstraint(translationMetersPerSecond)
          .withAngularSpeedConstraint(angularRadiansPerSecond);
    }

    public DriveCommandBuilder usePercentSpeeds(boolean enabled) {
      this.m_usePercentSpeeds = Optional.of(enabled);
      return this;
    }

    private Supplier<Boolean> getFieldRelativeSupplier() {
      return m_useFieldRelative.orElse(() -> kDefaultUseFieldRelative);
    }

    private Supplier<Double> getXSpeedSupplier() {
      return m_xSpeedSupplier.orElse(kDefaultDoubleSupplier);
    }

    private Supplier<Double> getYSpeedSupplier() {
      return m_ySpeedSupplier.orElse(kDefaultDoubleSupplier);
    }

    private Supplier<Double> getAngularSpeedSupplier() {
      return m_angularSpeedSupplier.orElse(kDefaultDoubleSupplier);
    }

    private double getTranslationalSpeedConstraint() {
      return m_translationalSpeedConstraint.orElse(kDefaultTranslationalSpeedConstraint);
    }

    private double getAngularSpeedConstraint() {
      return m_angularSpeedConstraint.orElse(kDefaultAngularSpeedConstraint);
    }

    private boolean usePercentSpeeds() {
      return m_usePercentSpeeds.orElse(kDefaultUsePercentSpeeds);
    }

    private double getCalculatedSpeed(double input, boolean usePercent, double maxSpeed) {
      if (usePercent) {
        return MathUtil.clamp(input, -1.0, 1.0) * maxSpeed;
      }

      return MathUtil.clamp(input, -maxSpeed, maxSpeed);
    }
  }
}
