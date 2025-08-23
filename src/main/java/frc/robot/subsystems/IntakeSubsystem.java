package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    SparkMax motorLeft;
    SparkMax motorRight;

    public IntakeSubsystem() {
        motorLeft = new SparkMax(15, MotorType.kBrushless);
        motorRight = new SparkMax(17, MotorType.kBrushless);

        SparkMaxConfig globalConfigleft = new SparkMaxConfig();
        SparkMaxConfig globalConfigright = new SparkMaxConfig();

        globalConfigleft
        .smartCurrentLimit(50)
        .idleMode(IdleMode.kBrake);

        globalConfigright
        .apply(globalConfigleft)
        .inverted(true);

    }

    public void setMotores(double velocidade) {
        motorLeft.set(velocidade);
        motorRight.set(velocidade);

    }

    public void stopMotores() {
        this.setMotores(0);
    }

    public Command rodar() {
        return Commands.startEnd(
            () -> this.setMotores(-0.4), 
            this::stopMotores,
            this);
    }
}
