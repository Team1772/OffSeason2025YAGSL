package com.example.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public class Wheel {
    private final PWMSparkMax m_motor;
    private final Encoder m_encoder;
    private final PIDController m_pidController;
    
    private double m_targetAngle = 0.0;
    private double m_currentAngle = 0.0;
    
    // Logging para AdvantageScope
    private final DoubleLogEntry m_targetAngleLog;
    private final DoubleLogEntry m_currentAngleLog;
    private final DoubleLogEntry m_motorOutputLog;
    private final DoubleLogEntry m_errorLog;

    public Wheel() {
        // Configuração do motor (porta PWM 0)
        m_motor = new PWMSparkMax(0);
        
        // Configuração do encoder (portas DIO 0 e 1)
        m_encoder = new Encoder(0, 1);
        m_encoder.setDistancePerPulse(360.0 / 1024.0); // 360 graus por 1024 pulsos
        
        // Configuração do PID Controller
        m_pidController = new PIDController(0.01, 0.0, 0.001);
        m_pidController.setTolerance(2.0); // Tolerância de 2 graus
        m_pidController.enableContinuousInput(-180, 180);
        
        // Configuração do logging
        DataLog log = DataLogManager.getLog();
        m_targetAngleLog = new DoubleLogEntry(log, "/wheel/target_angle");
        m_currentAngleLog = new DoubleLogEntry(log, "/wheel/current_angle");
        m_motorOutputLog = new DoubleLogEntry(log, "/wheel/motor_output");
        m_errorLog = new DoubleLogEntry(log, "/wheel/error");
    }

    public void setTargetAngle(double angle) {
        m_targetAngle = angle;
        m_pidController.setSetpoint(angle);
    }

    public double getAngle() {
        return m_currentAngle;
    }

    public double getTargetAngle() {
        return m_targetAngle;
    }

    public boolean atSetpoint() {
        return m_pidController.atSetpoint();
    }

    public void update() {
        // Lê o ângulo atual do encoder
        m_currentAngle = m_encoder.getDistance();
        
        // Calcula a saída do PID
        double output = m_pidController.calculate(m_currentAngle);
        
        // Limita a saída do motor
        output = Math.max(-1.0, Math.min(1.0, output));
        
        // Define a velocidade do motor
        m_motor.set(output);
        
        // Calcula o erro
        double error = m_targetAngle - m_currentAngle;
        
        // Logging para AdvantageScope
        m_targetAngleLog.append(m_targetAngle);
        m_currentAngleLog.append(m_currentAngle);
        m_motorOutputLog.append(output);
        m_errorLog.append(error);
        
        // Dados para SmartDashboard
        SmartDashboard.putNumber("Wheel/Target Angle", m_targetAngle);
        SmartDashboard.putNumber("Wheel/Current Angle", m_currentAngle);
        SmartDashboard.putNumber("Wheel/Motor Output", output);
        SmartDashboard.putNumber("Wheel/Error", error);
        SmartDashboard.putBoolean("Wheel/At Setpoint", atSetpoint());
        
        // Dados para análise de overshoot/undershoot
        SmartDashboard.putNumber("Wheel/PID_P", m_pidController.getP());
        SmartDashboard.putNumber("Wheel/PID_I", m_pidController.getI());
        SmartDashboard.putNumber("Wheel/PID_D", m_pidController.getD());
    }
    
    // Métodos para ajustar PID em tempo real
    public void setPIDConstants(double kP, double kI, double kD) {
        m_pidController.setPID(kP, kI, kD);
    }
    
    public void stop() {
        m_motor.set(0.0);
    }
}
