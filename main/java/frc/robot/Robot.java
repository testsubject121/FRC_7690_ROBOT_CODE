package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;

public class Robot extends TimedRobot {
	/** motor controllers and joystick(s) */
	VictorSPX _leftMaster = new VictorSPX(3);
	VictorSPX _rightMaster = new VictorSPX(1);
	VictorSPX rs = new VictorSPX(2);
	VictorSPX ls = new VictorSPX(4);
	Joystick _gamepad = new Joystick(0);
	Joystick turnStick = new Joystick(0);

	//TEMP TEMP
	Victor windowMotor = new Victor(0);
	AnalogInput ai = new AnalogInput(0);

	//pnematics objects
	Compressor compressor;
	DoubleSolenoid hatchDisconnectSolenoid = new DoubleSolenoid(0, 1);

	@Override
	public void robotInit() {

		//temp stuff
		windowMotor.enableDeadbandElimination(true);
		windowMotor.set(100);
		//starts up the camera!
		CameraServer.getInstance().startAutomaticCapture();
	}
	
	@Override
	public void teleopInit(){
		
		compressor = new Compressor();
		//closed loop control enables the pressure sensor and compressor
		compressor.setClosedLoopControl(false);

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

	}
	
	@Override
	public void teleopPeriodic() {	
		//gets the volatge from the prox sensor
		double voltage = ai.getVoltage()/0.009766;	
		double forward;
		double turn;

		/* Gamepad processing */

		//the code below takes the distance from the proximity sensor and determines whether to slow the robot down or not
		//change the value below to change the distance (30 - 60)
		if(voltage < 60){
			System.out.println(voltage);
			 forward = -1 * _gamepad.getRawAxis(1)/5;
			 if(forward < 0) forward *= 5;
			 turn = _gamepad.getRawAxis(4)/5;
		}else{
			 forward = -1 * _gamepad.getRawAxis(1);
			 turn = _gamepad.getRawAxis(4);	
		}


		forward = Deadband(forward);
		turn = Deadband(turn);

		/* Arcade Drive using PercentOutput along with Arbitrary Feed Forward supplied by turn */
		_leftMaster.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);
		_rightMaster.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, +turn);
		ls.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);
		rs.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, +turn);

		//pneumatics buttons
		boolean triggerPressed = _gamepad.getRawButton(1);
		boolean compButton = _gamepad.getRawButton(3);

		//temp 
		boolean button11 = _gamepad.getRawButton(11);
		boolean button10 = _gamepad.getRawButton(10);

	
		//if the trigger is pressed:
		if(triggerPressed){
			//open extend the pistons
			hatchDisconnectSolenoid.set(DoubleSolenoid.Value.kReverse);
		}else{
			//retract the pistons
			hatchDisconnectSolenoid.set(DoubleSolenoid.Value.kForward);
		}
		//if the compressor enable button is pressed:
		if(compButton){
			//enable the compressor
			compressor.setClosedLoopControl(true);
		}else{
			//disable the compressor
			compressor.setClosedLoopControl(false);
		}
		//if the button is pressed for a while, the compressor will automatically shut off
		//because of the pressure is near 120 PSI

		if(button10){
			windowMotor.set(1);
		}else if(button11){
			windowMotor.set(-1);
		}else{
			windowMotor.set(0);
		}


	}

	/** Deadband 3 percent, used on the gamepad */
	double Deadband(double value) {
		/* Upper deadband */
		if (value >= +0.03) 
			return value;
		
		/* Lower deadband */
		if (value <= -0.03)
			return value;
		
		/* Outside deadband */
		return 0;
	}

	//this method scales the joystick output so that the robot moves
	//slower when the joystick is barely moved, but
	//allows for full power
	double scale(double value){
		if(value >= -0.9 && value <= 0.9){
			value = Math.pow(value, 2);
		}

		return value;
	}

}
