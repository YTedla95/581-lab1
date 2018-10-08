package lab1;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;

// ---- Lab1 ---- Ben Trybulski (bentry), Johanns Tedla (pid) 
//
// Objective 1 (Odometry): 
//   Upon pressing center gray button, the robot moves forward for 1.2m, stops, then beeps.
//   It remains stopped until the center button is pressed again. 
//   - 10 points for distance traveled, 10 points for minimizing deviation
//
// Objective 2 (Ranging): 
//   The robot continues moving forward until it is 50cm away from the >=17cm wall ahead of it. 
//   (50cm is measured from the closest point on the robot). Use the ultrasonic sensor for this. 
//   It will beep and stop again it reaches that point. 
//   - 10 points for distance from wall, 10 points for minimizing deviation
//
// Objective 3 (Bumping): 
//   After pressing the center button, the robot moves forward until it bumps into the wall. 
//   Then it reverses 50cm away from the wall and stops again. 
//  - 10 points for bumping wall and reversing, 5 for correct distance, 5 for deviation from center

public class Lab1FunctionalRunner {
	public static void obj1() {
		EV3LargeRegulatedMotor mA = new EV3LargeRegulatedMotor(MotorPort.A);
		EV3LargeRegulatedMotor mB = new EV3LargeRegulatedMotor(MotorPort.D);
		mA.setSpeed(720);
		mB.setSpeed(720);
		// Wheels need to rotate 2455.533407703528 degrees to move 1.2 meters
		// or perform 6.8209261325098005 rotations 
		// -- moves forward for 3.41046 seconds at 2 rotations per second
		long t = System.currentTimeMillis();
		long end = t + 3411;
		while (System.currentTimeMillis() < end) {
				mA.startSynchronization();
				mA.backward();
				mB.backward();
				mA.endSynchronization();
		}
		mA.close();
		mB.close();
		Sound.beep();
	}

	public static void obj2() {
		EV3LargeRegulatedMotor mA = new EV3LargeRegulatedMotor(MotorPort.A);
		EV3LargeRegulatedMotor mB = new EV3LargeRegulatedMotor(MotorPort.D);
		mA.setSpeed(720);
		mB.setSpeed(720);
		EV3UltrasonicSensor ultraSensor = new EV3UltrasonicSensor(SensorPort.S1);
		SensorMode sonic = (SensorMode) ultraSensor.getDistanceMode();
		float[] sampleSonic = new float[sonic.sampleSize()];
		do {
			sonic.fetchSample(sampleSonic, 0);
			System.out.println("Sonic: " + sampleSonic[0]);
			mA.startSynchronization();
			mA.backward();
			mB.backward();
			mA.endSynchronization();
			
		} while (sampleSonic[0] > 0.5);
		ultraSensor.close();
		mA.close();
		mB.close();
		Sound.beep();
	}
	
	public static void obj3() {
		EV3LargeRegulatedMotor mA = new EV3LargeRegulatedMotor(MotorPort.A);
		EV3LargeRegulatedMotor mB = new EV3LargeRegulatedMotor(MotorPort.D);
		mA.setSpeed(720);
		mB.setSpeed(720);
		EV3TouchSensor touchSensor = new EV3TouchSensor(SensorPort.S2);
		SensorMode touch = (SensorMode) touchSensor.getTouchMode();
		float[] sample = new float[touch.sampleSize()];
		do {
			touch.fetchSample(sample, 0);
			System.out.println(sample[0]);
			mA.startSynchronization();
			mA.backward();
			mB.backward();
			mA.endSynchronization();
		} while (sample[0] == 0);
		
		// 50cm = 2.8420525552124167 rotations forwards
		// --> 1.421 seconds at 2 rotations per second
		long t = System.currentTimeMillis();
		long end = t + 1421;
		while (System.currentTimeMillis() < end) {
				mA.startSynchronization();
				mA.forward();
				mB.forward();
				mA.endSynchronization();
		}
		touchSensor.close();
		mA.close();
		mB.close();
		Sound.beep();
	}
	
	public static void moveMotors() {
		EV3LargeRegulatedMotor mA = new EV3LargeRegulatedMotor(MotorPort.A);
		EV3LargeRegulatedMotor mB = new EV3LargeRegulatedMotor(MotorPort.D);
		int motorA[] = {0,360,720,1080,360,0,180,0};
		int motorB[] = {0,360,720,1080,360,0,180,0};
		mA.setSpeed(720); mB.setSpeed(720);
		for (int i = 0; i < motorA.length; ++i) {
			mA.startSynchronization();
			mA.rotateTo(motorA[i]); mB.rotateTo(motorB[i]);
			mA.endSynchronization();
			mA.waitComplete();
			mB.waitComplete();
		}
	}
	
	public static void run() {
		System.out.println("Running lab1...");
		Button.waitForAnyPress();
		obj1();
		System.out.println("Obj1 completed");
		obj2();
		System.out.println("Obj2 completed");
		obj3();
		System.out.println("Obj3 completed");
		System.out.println("...finished lab1");
	}
	
	public static void main(String[] args) {
		run();
	}
}