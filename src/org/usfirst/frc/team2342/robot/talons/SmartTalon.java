package org.usfirst.frc.team2342.robot.talons;

import com.ctre.CANTalon;

/**
 * General wrapper class for CANTalons. Supports voltage, velocity, and position talon modes.
 * @author cooli
 *
 */
public class SmartTalon extends CANTalon {
	
	/*
	 * translates between TalonControlMode enum and user mode input via:
	 * TalonControlMode mode = user input mode + MODE_OFFSET
	 */
	private static final int MODE_OFFSET = 4;
	
	//put a minus sign in front of all setpoints,
	//used for reversed-polarity talons and devices
	private boolean inverted;

	//maximum forward and reverse speeds
	private double maxForwardSpeed;
	private double maxReverseSpeed;
	
	//current setpoint
	private double goal;
	
	/* 
	 * current mode, offset by MODE_OFFSET
	 * 0: VOLTAGE MODE
	 * 1: POSITION MODE
	 * 2: VELOCITY MODE
	 */
	private int mode;
	
	//PID gains for velocity and distance
	private PIDGains velocityGains;
	private PIDGains distanceGains;
	
	public SmartTalon(int deviceNumber) {
		this(deviceNumber, false, 0);
	}
	
	public SmartTalon(int deviceNumber, boolean inverted, int initialMode)
	{
		super(deviceNumber);
		this.inverted = inverted;
		
		maxForwardSpeed = 1.0;
		maxReverseSpeed = 1.0;
		
		velocityGains = new PIDGains(0,0,0,0,0,0);
		distanceGains = new PIDGains(0,0,0,0,0,0);
		mode = initialMode + MODE_OFFSET;
		
		if(initialMode == 0)
			setToVelocity();
		else if(initialMode == 1)
			setToDistance();
		else if(initialMode == 2)
			setToVelocity();
		
		setFeedbackDevice(FeedbackDevice.QuadEncoder);
	}
	
	public SmartTalon(int deviceNumber, boolean inverted, int initialMode, FeedbackDevice device)
	{
		this(deviceNumber, inverted, initialMode);
		
		setFeedbackDevice(device);
	}
	
	private void setToVelocity()
	{
		setP(velocityGains.getP());
		setI(velocityGains.getI());
		setD(velocityGains.getD());
		setIZone(velocityGains.getIzone());
		setF(velocityGains.getFf());
		setVoltageRampRate(velocityGains.getRr());
	}

	private void setToDistance()
	{
		setP(distanceGains.getP());
		setI(distanceGains.getI());
		setD(distanceGains.getD());
		setIZone(distanceGains.getIzone());
		setF(distanceGains.getFf());
		setVoltageRampRate(distanceGains.getRr());
	}
	
	/*
	 * Go at a speed using velocity gains
	 */
	public void goAt(double speed)
	{
		speed = (speed > 1) ? 1 : speed;
		speed = (speed < -1) ? -1 : speed;
		
		speed = (speed > 0) ? speed * maxForwardSpeed : speed * maxReverseSpeed;
	
		if(mode != TalonControlMode.Speed.getValue()) {
			setToVelocity();
			changeControlMode(TalonControlMode.Speed);
			mode = TalonControlMode.Speed.getValue();
		}
			
		configMaxOutputVoltage(12);
		
		if(!inverted)
			setSetpoint(speed);
		else
			setSetpoint(-speed);
	}
	
	/*
	 * Go at a specific voltage, independent of all PID gains
	 */
	public void goVoltage(double speed)
	{
		speed = (speed > 1) ? 1 : speed;
		speed = (speed < -1) ? -1 : speed;
		
		if(mode != TalonControlMode.PercentVbus.getValue()) {
			changeControlMode(TalonControlMode.PercentVbus);
			mode = TalonControlMode.PercentVbus.getValue();
		}
		
		configMaxOutputVoltage(12);
		
		if(inverted)
			set(-speed);
		else
			set(speed);
	}
	
	/*
	 * Go a specific distance, using distance PID gains
	 */
	public void goDistance(double distance, double speed)
	{
		speed = (speed > 1) ? 1 : speed;
		speed = (speed < -1) ? -1 : speed;
		
		double setPoint = getPosition() + distance;
		
		if(mode != TalonControlMode.Position.getValue()) {
			setToDistance();
			changeControlMode(TalonControlMode.Position);
			mode = TalonControlMode.Position.getValue();
		}
		
		configMaxOutputVoltage(12 * speed);
		
		if(!inverted)
			setSetpoint(setPoint);
		else
			setSetpoint(-setPoint);
	}
	
	public double getMaxForwardSpeed() {
		return maxForwardSpeed;
	}

	public void setMaxForwardSpeed(double maxFowardSpeed) {
		this.maxForwardSpeed = maxFowardSpeed;
	}

	public double getMaxReverseSpeed() {
		return maxReverseSpeed;
	}

	public void setMaxReverseSpeed(double maxReverseSpeed) {
		this.maxReverseSpeed = maxReverseSpeed;
	}

	public double getGoal() {
		return goal;
	}

	public void setGoal(double goal) {
		this.goal = goal;
	}

	public boolean isInverted() {
		return inverted;
	}

	public int getMode() {
		return mode - MODE_OFFSET;
	}

	public void setInverted(boolean inverted) {
		this.inverted = inverted;
	}

	public void setMaxForwardSpeed1(double maxForwardSpeed) {
		this.maxForwardSpeed = maxForwardSpeed;
	}

	public double getmaxReverseSpeed() {
		return maxReverseSpeed;
	}

	public void setmaxReverseSpeed(double maxReverseSpeed) {
		this.maxReverseSpeed = maxReverseSpeed;
	}

	public double getgoal() {
		return goal;
	}

	public void setgoal(double goal) {
		this.goal = goal;
	}

	public int getmode() {
		return mode;
	}

	public void setmode(int mode) {
		this.mode = mode;
	}

	public PIDGains getvelocityGains() {
		return velocityGains;
	}

	public void setvelocityGains(PIDGains velocityGains) {
		this.velocityGains = velocityGains;
	}

	public PIDGains getdistanceGains() {
		return distanceGains;
	}

	public void setdistanceGains(PIDGains distanceGains) {
		this.distanceGains = distanceGains;
	}
	
	

}
