package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;

public class Robot extends TimedRobot {
	/** Hardware, either Talon could be a Victor */
	VictorSPX _leftMaster = new VictorSPX(3);
	VictorSPX _rightMaster = new VictorSPX(1);
	VictorSPX rs = new VictorSPX(2);
	VictorSPX ls = new VictorSPX(4);
	Joystick _gamepad = new Joystick(0);

	//TEMP TEMP
	Victor windowMotor = new Victor(1);

	//pnematics
	Compressor compressor;
	DoubleSolenoid hatchDisconnectSolenoid = new DoubleSolenoid(0, 1);

	@Override
	public void robotInit() {
		windowMotor.enableDeadbandElimination(true);
		windowMotor.set(100);
		CameraServer.getInstance().startAutomaticCapture();
	}
	
	@Override
	public void teleopInit(){
		compressor = new Compressor();
		compressor.setClosedLoopControl(true);
		/* Ensure motor output is neutral during init */
		_leftMaster.set(ControlMode.PercentOutput, 0);
		_rightMaster.set(ControlMode.PercentOutput, 0);
		ls.set(ControlMode.PercentOutput,0);
		rs.set(ControlMode.PercentOutput,0);


		/* Factory Default all hardware to prevent unexpected behaviour */
		_leftMaster.configFactoryDefault();
		_rightMaster.configFactoryDefault();
		ls.configFactoryDefault();
		rs.configFactoryDefault();

		
		/* Set Neutral mode */
		_leftMaster.setNeutralMode(NeutralMode.Brake);
		_rightMaster.setNeutralMode(NeutralMode.Brake);
		ls.setNeutralMode(NeutralMode.Brake);
		rs.setNeutralMode(NeutralMode.Brake);
		
		
		/* Configure output direction */
		_leftMaster.setInverted(true);
		ls.setInverted(true);
		_rightMaster.setInverted(false);
		rs.setInverted(false);
		
		System.out.println("This is Arcade Drive using Arbitrary Feed Forward.");
	}
	
	@Override
	public void teleopPeriodic() {		
		/* Gamepad processing */
		double forward = -1 * _gamepad.getY();
		double turn = _gamepad.getX();		
		forward = Deadband(forward);
		turn = Deadband(turn);

		/* Arcade Drive using PercentOutput along with Arbitrary Feed Forward supplied by turn */
		_leftMaster.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, +turn);
		_rightMaster.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);
		ls.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, +turn);
		rs.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);

		//pneumatics
		boolean triggerPressed = false;
		boolean sideButtonPressed = false;
		triggerPressed = _gamepad.getRawButton(1);
		sideButtonPressed = _gamepad.getRawButton(2);
		boolean compButton = _gamepad.getRawButton(3);
		boolean button11 = _gamepad.getRawButton(11);
		boolean button10 = _gamepad.getRawButton(10);

		if(!(triggerPressed && sideButtonPressed)){
			if(triggerPressed) hatchDisconnectSolenoid.set(DoubleSolenoid.Value.kForward);
			else if(sideButtonPressed) hatchDisconnectSolenoid.set(DoubleSolenoid.Value.kReverse);
		}
		if(compButton){
			compressor.setClosedLoopControl(true);
		}else{
			compressor.setClosedLoopControl(false);
		}

		if(button10){
			windowMotor.set(1);
		}else if(button11){
			windowMotor.set(-1);
		}else{
			windowMotor.set(0);
		}


	}

	/** Deadband 5 percent, used on the gamepad */
	double Deadband(double value) {
		/* Upper deadband */
		if (value >= +0.05) 
			return value;
		
		/* Lower deadband */
		if (value <= -0.05)
			return value;
		
		/* Outside deadband */
		return 0;
	}
}
