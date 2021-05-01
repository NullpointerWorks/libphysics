package com.nullpointerworks.test.model;

import static com.nullpointerworks.physics.engine.VectorMath.mul;
import static com.nullpointerworks.physics.engine.VectorMath.project;
import static com.nullpointerworks.physics.engine.Collision.solve;
import static com.nullpointerworks.physics.engine.ImpulseMath.getRestingConstant;
import static com.nullpointerworks.physics.engine.ImpulseMath.equal;

import java.util.ArrayList;
import java.util.List;
import java.util.Vector;

import com.nullpointerworks.game.Asap;
import com.nullpointerworks.physics.engine.Composite;
import com.nullpointerworks.physics.engine.manifold.Manifold;

public class PhysicsLoop extends Asap
{
	private List<Composite> bodies;
	private List<Manifold> contacts;
	
	private float[] gravityVector;
	private int iterations = 10;
	
	public PhysicsLoop()
	{
		bodies = new Vector<Composite>();
		contacts = new ArrayList<Manifold>();
		gravityVector = new float[] {0f, 9.8f};
	}
	
	public synchronized void setIntegrationCycles(int it)
	{
		if (it < 1) it = 1;
		iterations = it;
	}
	
	public synchronized void setGravity(float[] vector, float acc)
	{
		gravityVector = mul(vector,  acc);
	}
	
	public synchronized void addComposite(Composite c)
	{
		bodies.add(c);
	}
	
	public synchronized List<Composite> getBodies()
	{
		List<Composite> copy = new ArrayList<Composite>();
		for (Composite c : bodies)
		{
			copy.add(c.getCopy());
		}
		return copy;
	}
	
	// =======================================================================================
	
	@Override
	public synchronized void onUpdate(double dt) 
	{
		contacts.clear();
		
		float resting = getRestingConstant(gravityVector, (float)dt);
		
		int lb = bodies.size();
		for (int a=0; a<lb; a++)
		{
			Composite A = bodies.get(a);
			for (int b=a+1; b<lb; b++)
			{
				Composite B = bodies.get(b);
				
				// check blacklist, immovability or "infinite" mass
				if (A.contains(B)) continue;
				if (B.contains(A)) continue;
				if (A.isImmovable() && B.isImmovable()) continue;
				if (equal(A.inv_mass + B.inv_mass, 0f)) continue;
				
				Manifold m = new Manifold(A, B, resting);
				solve(m, A, B);
				
				if (m.contact_count > 0)
				{
					contacts.add(m);
				}
			}
		}
		
		// ==== apply contact forces and accelerations
		for (Composite C : bodies)
		{
			integrateForces(C, (float)dt);
		}
		
		// ==== apply impulses
		for (Manifold m : contacts)
		{
			m.preprare();
		}
		for (int i=0; i<iterations; i++)
		for (Manifold m : contacts)
		{
			m.applyImpulse();
		}
		
		// ==== apply velocities
		for (Composite C : bodies)
		{
			integrateVelocity(C, (float)dt);
		}
		
		for (Manifold m : contacts)
		{
			m.applyCorrection();
		}
		
		// ==== wipe forces for next update
		for (Composite C : bodies)
		{
			C.clear();
		}
	}
	
	@Override
	public void onInit() 
	{
		bodies.clear();
		contacts.clear();
	}
	
	@Override
	public void onRender(double interpolation) { }

	@Override
	public void onDispose() 
	{
		bodies.clear();
		contacts.clear();
		bodies = null;
		contacts = null;
	}
	
	/**
	 * apply all accelerations to the composite's velocity
	 */
	private void integrateForces(Composite b, float dt)
	{
		if (b.isImmovable()) return;
		
		float[] v 	= b.getLinearMotion().getVelocity();
		float a 	= b.getAngularMotion().getVelocity();
		
		// calculate change in velocity
		// dv = F/m * dt = force divided by mass times delta-time
		v = project(v, b.force, b.inv_mass * dt);
		v = project(v, gravityVector, dt);
		
		// calculate change in angular velocity 
		// w = T/Ip * dt = torque divided by moment of inertia times delta-time
		a = a + b.torque * b.inv_inertia * dt;
		
		b.getLinearMotion().setVelocity(v);
		b.getAngularMotion().setVelocity(a);
	}
	
	/**
	 * apply all velocities to the composite's position
	 */
	private void integrateVelocity(Composite b, float dt)
	{
		if (b.isImmovable()) return;
		
		// Euler integration
		integrateForces(b,dt);
		
		float[] lP 	= b.getLinearMotion().getPosition();
		float[] lV	= b.getLinearMotion().getVelocity();
		
		float aO 	= b.getAngularMotion().getOrientation();
		float aV	= b.getAngularMotion().getVelocity();
		
		// p = v * dt
		lP = project(lP, lV, dt);
		aO = aO + aV * dt;
		
		b.getLinearMotion().setPosition(lP);
		b.getAngularMotion().setOrientation(aO);
		
		// semi-implicit Euler by applying forces after integration
		//integrateForces(b,dt);
	}
}
