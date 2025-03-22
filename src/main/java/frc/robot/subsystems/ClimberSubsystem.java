package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

// Phoenix 6 imports
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

@SuppressWarnings("unused")

public class ClimberSubsystem extends SubsystemBase {

    // Motor & control request object
    private final TalonFX climbMotor;
    private final VoltageOut voltageRequest;
    private final PositionVoltage positionRequest;

    private boolean servoExtended = true;

    public ClimberSubsystem() {
        // Initialize motor
        climbMotor = new TalonFX(ClimberConstants.CLIMBER_MOTOR_ID);

        // Create a Voltage control request initially set to 0 V
        voltageRequest = new VoltageOut(0);
        
        positionRequest = new PositionVoltage(0).withSlot(0);
    
        configureClimberMotor();

        }

        public void configureClimberMotor() {
            
            SoftwareLimitSwitchConfigs softLimitConfig = new SoftwareLimitSwitchConfigs();
            softLimitConfig.ForwardSoftLimitEnable = false;
            // softLimitConfig.ForwardSoftLimitThreshold = ClimbConstants.FORWARD_SOFT_LIMIT;
            softLimitConfig.ReverseSoftLimitEnable = false;
            // softLimitConfig.ReverseSoftLimitThreshold = ClimbConstants.REVERSE_SOFT_LIMIT;
            climbMotor.getConfigurator().apply(softLimitConfig);
        }

    // Output to lift the robot up
    public void climbUp() {
        climbMotor.setControl(voltageRequest.withOutput(ClimberConstants.CLIMB_VOLTAGE * -1));
    }

    // Output to lower the robot down
    public void climbDown() {
        climbMotor.setControl(voltageRequest.withOutput(ClimberConstants.CLIMB_VOLTAGE));
    }

    public void stopClimb() {
        climbMotor.setControl(voltageRequest.withOutput(0.0));
    }

    public void holdClimb() {
        climbMotor.setControl(voltageRequest.withOutput(0.5));
    }
    
    @Override
    public void periodic() {

        SmartDashboard.putNumber("Climber/MotorPosition", climbMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Climber/MotorVoltage", climbMotor.getSupplyVoltage().getValueAsDouble());
        
    }

} // end of class