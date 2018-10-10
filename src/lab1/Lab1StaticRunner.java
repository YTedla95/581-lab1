package lab1;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.utility.Delay;

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

public class Lab1StaticRunner {
	static EV3LargeRegulatedMotor mA;
	static EV3LargeRegulatedMotor mB;
	static EV3UltrasonicSensor ultraSensor;
	static EV3TouchSensor touchSensor;
	static SensorMode sonic;
	static SensorMode touch;
	
	public Lab1StaticRunner() {}
	
	public void obj1() {
		// Wheels need to rotate 2455.533407703528 degrees to move 1.2 meters
		// or perform 6.8209261325098005 rotations 
		// -- moves forward for 3.41046 seconds at 2 rotations per second
		long t = System.currentTimeMillis();
		long end = t + 6821; // 1 rps
		while (System.currentTimeMillis() < end) {
				mA.startSynchronization();
				mA.backward();
				mB.backward();
				mA.endSynchronization();
		}
		mA.startSynchronization();
		mA.stop();
		mB.stop();
		mA.endSynchronization();
	}

	public void obj2() {
		float[] sampleSonic = new float[sonic.sampleSize()];
		do {
			sonic.fetchSample(sampleSonic, 0);
			System.out.println("Sonic: " + sampleSonic[0]);
			mA.startSynchronization();
			mA.backward();
			mB.backward();
			mA.endSynchronization();
			Delay.msDelay(750);	
		} while (sampleSonic[0] > 0.65); // 0.5 + err for read delay 
		mA.startSynchronization();
		mA.stop();
		mB.stop();
		mA.endSynchronization();
	}
	
	public void obj3() {
		float[] sample = new float[touch.sampleSize()];
		do {
			touch.fetchSample(sample, 0);
			System.out.println(sample[0]);
			mA.startSynchronization();
			mA.backward();
			mB.backward();
			mA.endSynchronization();
			Delay.msDelay(750);
		} while (sample[0] == 0);
		
		// 50cm = 2.8420525552124167 rotations forwards
		// --> 1.421 seconds at 2 rotations per second
		long t = System.currentTimeMillis();
		long end = t + 2842;
		while (System.currentTimeMillis() < end) {
				mA.startSynchronization();
				mA.forward();
				mB.forward();
				mA.endSynchronization();
		}
		mA.startSynchronization();
		mA.stop();
		mB.stop();
		mA.endSynchronization();
	}
	
	public void go() {
		Sound.setVolume(100);
		System.out.println("Running lab1...");
		Button.waitForAnyPress();

		obj1();
		Sound.beep();
		Delay.msDelay(3000);
		System.out.println("Obj1 completed");
		System.out.println("Press button for obj2");
		Button.waitForAnyPress();
		
		obj2();
		Sound.beep();
		Delay.msDelay(3000);
		System.out.println("Obj2 completed");
		System.out.println("Press button for obj3");
		Button.waitForAnyPress();
		
		obj3();
		Sound.beep();
		Delay.msDelay(3000);
		System.out.println("Obj3 completed");		
		System.out.println("...finished lab1");
	}
	
	public static void main(String[] args) {
		mA = new EV3LargeRegulatedMotor(MotorPort.A);
		mB = new EV3LargeRegulatedMotor(MotorPort.D);
		mA.setSpeed(360);
		mB.setSpeed(360);
		mA.synchronizeWith(new EV3LargeRegulatedMotor[] {mB}); 
		ultraSensor = new EV3UltrasonicSensor(SensorPort.S1);
		touchSensor = new EV3TouchSensor(SensorPort.S2);
		sonic = (SensorMode) ultraSensor.getDistanceMode();
		touch = (SensorMode) touchSensor.getTouchMode();
		Lab1StaticRunner lab1 = new Lab1StaticRunner();
		lab1.go();
	}
}