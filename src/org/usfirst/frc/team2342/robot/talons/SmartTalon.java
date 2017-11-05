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
	private boolean m_inverted;

	//maximum forward and reverse speeds
	private double m_maxForwardSpeed;
	private double m_maxReverseSpeed;
	
	//current setpoint
	private double m_goal;
	
	/* 
	 * current mode, offset by MODE_OFFSET
	 * 0: VOLTAGE MODE
	 * 1: POSITION MODE
	 * 2: VELOCITY MODE
	 */
	private int m_mode;
	
	//PID gains for velocity and distance
	private PIDGains m_velocityGains;
	private PIDGains m_distanceGains;
	
	public SmartTalon(int deviceNumber) {
		this(deviceNumber, false, 0);
	}
	
	public SmartTalon(int deviceNumber, boolean inverted, int initialMode)
	{
		super(deviceNumber);
		m_inverted = inverted;
		
		m_maxForwardSpeed = 1.0;
		m_maxReverseSpeed = 1.0;
		
		m_velocityGains = new PIDGains(0,0,0,0,0,0);
		m_distanceGains = new PIDGains(0,0,0,0,0,0);
		m_mode = initialMode + MODE_OFFSET;
		
		if(initialMode == 0)
			setToVelocity();
		else if(initialMode == 1)
			setToDistance();
		else if(initialMode == 2)
			setToVelocity();
	}
	
	/*
	public SmartTalon(int deviceNumber, FeedbackDevice device, boolean inverted) {
		super(deviceNumber);
		
		m_maxForwardSpeed = 1.0;
		m_maxReverseSpeed = 1.0;
		m_inverted = inverted;
		
		m_velocityGains = new PIDGains(0,0,0,0,0,0);
		m_distanceGains = new PIDGains(0,0,0,0,0,0);
		
		setToVelocity();
	}*/
	
	private void setToVelocity()
	{
		setP(m_velocityGains.getP());
		setI(m_velocityGains.getI());
		setD(m_velocityGains.getD());
		setIZone(m_velocityGains.getIzone());
		setF(m_velocityGains.getFf());
		setVoltageRampRate(m_velocityGains.getRr());
	}

	private void setToDistance()
	{
		setP(m_distanceGains.getP());
		setI(m_distanceGains.getI());
		setD(m_distanceGains.getD());
		setIZone(m_distanceGains.getIzone());
		setF(m_distanceGains.getFf());
		setVoltageRampRate(m_distanceGains.getRr());
	}
	
	/*
	 * Go at a speed using velocity gains
	 */
	public void goAt(double speed)
	{
		speed = (speed > 1) ? 1 : speed;
		speed = (speed < -1) ? -1 : speed;
		
		speed = (speed > 0) ? speed * m_maxForwardSpeed : speed * m_maxReverseSpeed;
	
		if(m_mode != TalonControlMode.Speed.getValue()) {
			setToVelocity();
			changeControlMode(TalonControlMode.Speed);
			m_mode = TalonControlMode.Speed.getValue();
		}
			
		configMaxOutputVoltage(12);
		
		if(!m_inverted)
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
		
		if(m_mode != TalonControlMode.PercentVbus.getValue()) {
			changeControlMode(TalonControlMode.PercentVbus);
			m_mode = TalonControlMode.PercentVbus.getValue();
		}
		
		configMaxOutputVoltage(12);
		
		if(m_inverted)
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
		
		if(m_mode != TalonControlMode.Position.getValue()) {
			setToDistance();
			changeControlMode(TalonControlMode.Position);
			m_mode = TalonControlMode.Position.getValue();
		}
		
		configMaxOutputVoltage(12 * speed);
		
		if(!m_inverted)
			setSetpoint(setPoint);
		else
			setSetpoint(-setPoint);
	}
	
	public double getMaxForwardSpeed() {
		return m_maxForwardSpeed;
	}

	public void setMaxForwardSpeed(double maxFowardSpeed) {
		this.m_maxForwardSpeed = maxFowardSpeed;
	}

	public double getMaxReverseSpeed() {
		return m_maxReverseSpeed;
	}

	public void setMaxReverseSpeed(double maxReverseSpeed) {
		this.m_maxReverseSpeed = maxReverseSpeed;
	}

	public double getGoal() {
		return m_goal;
	}

	public void setGoal(double goal) {
		this.m_goal = goal;
	}

	public boolean isInverted() {
		return m_inverted;
	}

	public int getMode() {
		return m_mode - MODE_OFFSET;
	}
	
	

}
