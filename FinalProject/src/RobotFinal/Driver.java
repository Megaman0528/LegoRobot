package RobotFinal;

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;

public class Driver {
	
	static EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(MotorPort.A);
	static EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(MotorPort.B);
	public static EV3MediumRegulatedMotor medMotor = new EV3MediumRegulatedMotor(MotorPort.C);
	static EV3UltrasonicSensor ultra = new EV3UltrasonicSensor(SensorPort.S1);
	static EV3TouchSensor touch = new EV3TouchSensor(SensorPort.S2);
	static EV3GyroSensor gyro = new EV3GyroSensor(SensorPort.S3);

	static SampleProvider myDistanceSample = ultra.getDistanceMode();
	static float[] distanceSample = new float[myDistanceSample.sampleSize()];

	static SampleProvider myTouchSample = touch.getTouchMode();
	static float[] touchSample = new float[myTouchSample.sampleSize()];

	static SampleProvider myAngleSample = gyro.getAngleMode();
	static float[] angleSample = new float[myAngleSample.sampleSize()];

	static int leftMotorTacho =0;
	static int rightMotorTacho =0;

	public static void main(String[] args) {
		LCD.drawString("Waiting for start...", 0, 0);
		
		ultra.enable();
		Button.waitForAnyPress();
        Move();
		findCup();
		Rotate();
	//	Return();
		Button.waitForAnyPress();
	}

	private static void findCup() {

		if (distanceSample[0] >= .05) {
			medMotor.rotate(-1800);
		}

	}

	public static void Rotate() {
		gyro.reset();
		myAngleSample.fetchSample(angleSample, 0);
		float myAngleReading = angleSample[0];
		float difference = 0;
		while (difference != 180) {
			LCD.drawString("" + difference, 0, 2);
			leftMotor.rotate(-360);
			difference = findDifference(myAngleReading);
			if (difference >= 180) {
				break;
			}
			rightMotor.rotate(360);
			difference = findDifference(myAngleReading);

			if (difference >= 180) {
				break;				
				
			}}}			

			
		
	

	private static float findDifference(float myAngleReading) {
		float difference;
		myAngleSample.fetchSample(angleSample, 0);
		float myNewReading = angleSample[0];
		difference = Math.abs(myAngleReading - myNewReading);
		return difference;
	}

	public static void Move() {

		while (distanceSample[0] <= .05) {
			
			if (TouchSense() == true) {
				rightMotor.rotate(300);
			}
			leftMotor.rotate(300);
			myDistanceSample.fetchSample(distanceSample, 0);
			if (distanceSample[0] >= .05) {
				
			
				break;
			}
			
		}
	}

	public static boolean TouchSense() {
		myTouchSample.fetchSample(touchSample, 0);
		
		
		return touchSample[0] == 1;
	}
	public static void Return() {
		leftMotor.synchronizeWith(new RegulatedMotor[] { Motor.B });
		leftMotor.startSynchronization();
		leftMotor.rotate(leftMotorTacho);
		Motor.B.rotate(rightMotorTacho);
		leftMotor.endSynchronization();	
		medMotor.rotate(1800);
	}

}
