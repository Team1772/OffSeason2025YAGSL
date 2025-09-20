package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ShooterSubsystem extends SubsystemBase {
    TalonFX motorLeft;
    TalonFX motorRight;


    public ShooterSubsystem() {
        motorLeft = new TalonFX(10); // Substitua pelo CAN ID correto
        motorRight = new TalonFX(11); // Substitua pelo CAN ID correto


        TalonFXConfiguration config = new TalonFXConfiguration();


        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.SupplyCurrentLimit = 50;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;


        // Aplica a configuração para ambos os motores
        motorLeft.getConfigurator().apply(config);
        motorRight.getConfigurator().apply(config);


        // Inverter motor direito se necessário
    }


    public void setMotores(double velocidade) {
        // Controle em percent output (-1.0 a 1.0)
        motorLeft.set(velocidade);
        motorRight.set(velocidade);
    }


    public void stopMotores() {
        setMotores(0);
    }


    public Command rodar() {
        return Commands.startEnd(
            () -> setMotores(0.8),  // velocidade de exemplo
            this::stopMotores,
            this
        );
    }
}

