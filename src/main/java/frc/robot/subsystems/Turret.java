package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.LimelightHelpers;
import frc.robot.config.TurretConfig;

@Logged
public class Turret extends SubsystemBase {
    protected TalonFX mTurretRotation;

    public Turret() {
        mTurretRotation = new TalonFX(1);

        TalonFXConfiguration turretRotationConfig = new TalonFXConfiguration();

        turretRotationConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        turretRotationConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        turretRotationConfig.CurrentLimits.SupplyCurrentLimit = TurretConfig.K_ROTATION_SUPPLY_CURRENT_LIMIT;
        turretRotationConfig.CurrentLimits.StatorCurrentLimit = TurretConfig.K_ROTATION_STATOR_CURRENT_LIMIT;

        turretRotationConfig.Slot0.kP = TurretConfig.K_ROTATION_P;
        turretRotationConfig.Slot0.kI = TurretConfig.K_ROTATION_I;
        turretRotationConfig.Slot0.kD = TurretConfig.K_ROTATION_D;
        turretRotationConfig.Slot0.kS = TurretConfig.K_ROTATION_S;
        turretRotationConfig.Slot0.kV = TurretConfig.K_ROTATION_V;
        turretRotationConfig.Slot0.kG = TurretConfig.K_ROTATION_G;
        turretRotationConfig.Slot0.kA = TurretConfig.K_ROTATION_A;

        turretRotationConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        mTurretRotation.getConfigurator().apply(turretRotationConfig);
    }

    final PositionVoltage m_request = new PositionVoltage(0);

    boolean flipping = false;

    public void moveToApriltag() {
        if (LimelightHelpers.getTV("limelight") && !flipping) {
            double currentPosition = mTurretRotation.getPosition().getValueAsDouble();
            double tx = -LimelightHelpers.getTX("limelight");
            double errorInRotations = (tx / 360.0) * TurretConfig.K_ROTATION_GEAR_RATIO;
            double targetPositon = currentPosition + errorInRotations;
            mTurretRotation.setControl(m_request.withPosition(targetPositon));

        }
    }

    public void flip() {
        if (mTurretRotation.getPosition().getValueAsDouble() >= 2) {
            flipping = true;
            stopMotors();
            mTurretRotation.setControl(m_request.withPosition(-1.9));
            new WaitCommand(2);
            flipping = false;

        }
        if (mTurretRotation.getPosition().getValueAsDouble() <= -2) {
            flipping = true;
            stopMotors();
            mTurretRotation.setControl(m_request.withPosition(1.9));
            new WaitCommand(2);
            flipping = false;
        }
    }

    public boolean shouldFlip() {
        if (mTurretRotation.getPosition().getValueAsDouble() >= 2 || mTurretRotation.getPosition().getValueAsDouble() <= -2) {
            return true;
        }
        else {
            return false;
        }
    }

    public void zeroTurret() {
        mTurretRotation.setPosition(0);
    }

    public void stopMotors() {
        mTurretRotation.stopMotor();
    }
}