package org.usfirst.frc.team1102.robot;
//Dennis Terry
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class IntakeClaw {
	
	VictorSP IntakeWheels;
	DoubleSolenoid Claw;
	
	public IntakeClaw(int wheels_port, int claw_port1, int claw_port2) {
		IntakeWheels = new VictorSP(wheels_port);
		Claw = new DoubleSolenoid(claw_port1, claw_port2);
	}
	
	public void setClaw(Value clawstate) {
		 Claw.set(clawstate);
	}
	
	public void setWheels(int state) {
		int in = 1;
		int out = 2;
		int out_slow = 3;
		
		if(state == in) {
			IntakeWheels.set(-0.8);
		}else {
			if(state == out) {
				IntakeWheels.set(0.8);
			}else {
				if(state == out_slow) {
					IntakeWheels.set(0.65);
				}else {
				IntakeWheels.set(0);
				}
			}
		}
	}
}
	
