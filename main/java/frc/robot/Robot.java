package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;

public class Robot extends TimedRobot {

	/** motor controllers and joystick(s) */
	VictorSPX _leftMaster = new VictorSPX(3);
	VictorSPX _rightMaster = new VictorSPX(1);
	VictorSPX rs = new VictorSPX(2);
	VictorSPX ls = new VictorSPX(4);
	Joystick _gamepad = new Joystick(0);

	//ramping
	double prevVal = 0;

	//periferals
	Victor legMotor =  new Victor(1);
	Victor ballIntake = new Victor(0);
	Relay lights = new Relay(0);
	Relay reverseLight = new Relay(1);
	Relay.Value lightsOn = Relay.Value.kForward;
	Relay.Value lightsOff = Relay.Value.kOff;

	//hatch light
	DigitalOutput hatchLight = new DigitalOutput(9);

	//reverse controls
	boolean circleButton;
	boolean triggerPressed;
	boolean reverseControls = false;
	double reverseControlDelay = 1;

	boolean triangle;
	boolean square;

	//bumbers
	DigitalInput leftBumber = new DigitalInput(0);
	DigitalInput rightBumber = new DigitalInput(1);
	DigitalInput hatchLimitSwitch = new DigitalInput(2);

	//rangefinder
	AnalogInput rangeFinder = new AnalogInput(0);

	//cameras
	UsbCamera frontCamera;
	UsbCamera rearCamera;

	//pnematics objects
	Compressor compressor;
	DoubleSolenoid hatchDisconnectSolenoid = new DoubleSolenoid(0, 1);
	DoubleSolenoid rearPistons = new DoubleSolenoid(2, 3);
	DoubleSolenoid frontPistons = new DoubleSolenoid(4, 5);

	//level 2 buttons
	boolean raiseFront = false;
	boolean raiseRear = false;
	boolean L1;
	boolean R1;


	/* *****************ROBOT INIT***************** */
	@Override
	public void robotInit() {

		lights.set(lightsOn);
		//starts up the camera!
		frontCamera = CameraServer.getInstance().startAutomaticCapture(0);
		rearCamera = CameraServer.getInstance().startAutomaticCapture(1);

	
		/* Ensure motor output is neutral during init */
		_leftMaster.set(ControlMode.PercentOutput, 0);
		_rightMaster.set(ControlMode.PercentOutput, 0);
		ls.set(ControlMode.PercentOutput,0);
		rs.set(ControlMode.PercentOutput,0);
		
		/* Configure output direction */
		_leftMaster.setInverted(true);
		ls.setInverted(true);
		_rightMaster.setInverted(false);
		rs.setInverted(false);

		//turns lights off
		lights.set(lightsOn);

	}
	
	/* *****************TELEOP INIT***************** */
	@Override
	public void teleopInit(){
	}
	
	/* *****************AUTO INIT***************** */
	@Override
	public void autonomousInit() {
		compressor = new Compressor();
		compressor.setClosedLoopControl(true);
	}

	/* *****************AUTO PERIODIC***************** */
	@Override
	public void autonomousPeriodic(){
		teleopPeriodic();
	}

	/* *****************TELEOP PERIODIC***************** */
	@Override
	public void teleopPeriodic() {
		
		hatchLight.set(true);

		double forward = 0;
		double turn = 0;
		///change if needed******************************************************************************
		//double threshold = 40000;
		//double distance = rangeFinder.getAverageVoltage() * 100;
		//System.out.println(distance);

		//distance sensor setting the lights on or off
		/*
		if(distance <= threshold){
			lights.set(lightsOn);
			System.out.println("ON");
		}else{
			lights.set(lightsOff);
		}
		*/

		//if bumpers are pressed, turn the lights on
		if(!leftBumber.get() || !rightBumber.get() || !hatchLimitSwitch.get()){
			lights.set(lightsOn);
		}else{
			lights.set(lightsOff);
		}

		//gamepad processing
			 forward = -1 * _gamepad.getRawAxis(1);
			 if(forward > 0){
				if((forward - prevVal) >= 0.07){
					forward = prevVal + 0.07;
				}
			 }else{
				if((forward - prevVal) <= -0.07){
					forward = prevVal - 0.07;
				}
			 }
			 turn = _gamepad.getRawAxis(4);	
			 prevVal=forward;

			
			 if(!reverseControls){
				 forward *= -1;
			 }
			 

		//adding a small deadzone 
		forward = Deadband(forward);
		turn = Deadband(turn);
		forward = scale(forward);
		turn = scale(turn);


		/* Arcade Drive using PercentOutput along with Arbitrary Feed Forward supplied by turn */

		
			_leftMaster.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, +turn);
			_rightMaster.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);
			ls.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, +turn);
			rs.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);

		//pneumatics buttons
		triggerPressed = _gamepad.getRawButton(1);
		circleButton = _gamepad.getRawButton(2);

		triangle = _gamepad.getRawButton(4);
		square = _gamepad.getRawButton(3);
 
		R1 = _gamepad.getRawButtonPressed(5);
		L1 = _gamepad.getRawButtonPressed(6);


		//SuperBallSucker buttons
		double L2 = _gamepad.getRawAxis(2);
		double R2 = _gamepad.getRawAxis(3);
		L2 = Deadband(L2)/2;
		R2 = Deadband(R2)/6;

		//if the trigger is pressed:
		if(triggerPressed){
			//open extend the pistons
			hatchDisconnectSolenoid.set(DoubleSolenoid.Value.kReverse);
		}else{
			//retract the pistons
			hatchDisconnectSolenoid.set(DoubleSolenoid.Value.kForward);
		}

		double foo = 0;
		if(triangle){
			foo += 0.25;
		}
		if(square){
			foo -=0.25;
		}
		legMotor.set(foo);

		//raise the front and back (toggle)
		if(R1) raiseFront = !raiseFront;
		if(L1) raiseRear = !raiseRear;

		if(raiseFront){
			frontPistons.set(DoubleSolenoid.Value.kReverse);
		}else{
			frontPistons.set(DoubleSolenoid.Value.kForward);
		}

		if(raiseRear){
			rearPistons.set(DoubleSolenoid.Value.kReverse);
		}else{
			rearPistons.set(DoubleSolenoid.Value.kForward);
		}

		//reverses the controls
		//add lights later?
		if(!reverseControls && circleButton && reverseControlDelay <= 0){
			reverseControls = !reverseControls;
			reverseControlDelay = 1;
			reverseLight.set(lightsOn);
		}else if(reverseControls && circleButton && reverseControlDelay <= 0){
			reverseControls = !reverseControls;
			reverseControlDelay = 1;
			reverseLight.set(lightsOff);
		}
		if(reverseControlDelay > 0) reverseControlDelay -= 0.04;


		ballIntake.set(L2 + (-1 * R2));

	}

	/** Deadband 3 percent, used on the gamepad */
	double Deadband(double value) {
		/* Upper deadband */
		if (value >= +0.02) 
			return value;
		
		/* Lower deadband */
		if (value <= -0.02)
			return value;
		
		/* Outside deadband */
		return 0;
	}

	//this method scales the joystick output so that the robot moves
	//slower when the joystick is barely moved, but
	//allows for full power
	double scale(double value){
		value *= -1;
		if(value >= -0.9 && value <= 0.9){
			if(value > 0){
				value = Math.pow(value, 2);
			}else{
				value = Math.pow(value, 2);
				value *= -1;
			}
			
		}

		return value;
	}

}
