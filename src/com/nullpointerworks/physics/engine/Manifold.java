package com.nullpointerworks.physics.engine;

import static com.nullpointerworks.physics.engine.VectorMath.create;

import com.nullpointerworks.math.FloatMath;
import com.nullpointerworks.math.vector.Vector2;
import com.nullpointerworks.physics.engine.material.Material;

public class Manifold 
{
	private final Vector2 vec2 = Vector2.New();
	public Composite A,B;
	private float RESTING;
	public int contact_count;
	public float penetration;
	public float restitution;
	public float sFriction;
	public float kFriction;
	public float[] normal;
	public float[][] contacts;
	
	public Manifold(Composite A, Composite B, float resting)
	{
		this.A = A; this.B = B;
		RESTING = resting;
		normal = new float[] {0f,0f};
		contacts = new float[][] {{0f,0f}, {0f,0f}};
	}
	
	/**
	 * Call the collision checker and solve for the given Compositions
	 */
	public void solve()
	{
		Collision.solve(this, A, B);
	}
	
	/**
	 * preliminary contact check
	 */
	public void preprare() 
	{
		Material matA = A.getMaterial();
		Material matB = B.getMaterial();
		
		// get average restitution
		restitution = FloatMath.min(matA.getRestitution(), matB.getRestitution());
		sFriction = FloatMath.pythagoras(matA.getStaticFriction(), matB.getStaticFriction());
		kFriction = FloatMath.pythagoras(matA.getKineticFriction(), matB.getKineticFriction());
		
		// for each contact point: see if its colliding, or resting on a surface
		for (int i=0; i<contact_count; i++)
		{
			float[] relativeA, relativeB, relativeV, contact;
			
			float[] Apos = A.getLinearMotion().getPosition();
			float[] Bpos = B.getLinearMotion().getPosition();
			
			float[] Avel = A.getLinearMotion().getVelocity();
			float[] Bvel = B.getLinearMotion().getVelocity();
			
			float Aaval = A.getAngularMotion().getVelocity();
			float Baval = B.getAngularMotion().getVelocity();
			
			contact = contacts[i];
			relativeA = vec2.sub(contact, Apos);
			relativeB = vec2.sub(contact, Bpos);
			
			relativeV = vec2.project(Bvel, vec2.normal(relativeB), Baval );
			relativeV = vec2.sub(relativeV, Avel);
			relativeV = vec2.project(relativeV, vec2.normal(relativeA), -Aaval );
			
			float m = vec2.dot(relativeV,relativeV); // square distance
			
			if (m < RESTING) restitution = 0.0f;
		}
	}
	
	/**
	 * 
	 */
	public void applyImpulse()
	{
		float inv_massA = A.inv_mass;
		float inv_massB = B.inv_mass;
		
		// check if the masses are infinite. Skip moving by impulse
		if (ImpulseMath.equal(inv_massA + inv_massB, 0.0f))
		{
			A.getLinearMotion().setVelocity( create(0f,0f) );
			B.getLinearMotion().setVelocity( create(0f,0f) );
			return;
		}
		
		// for each contact point
		for (int i=0; i<contact_count; i++)
		{
			float[] relativeA, relativeB, relativeV, contact, a, b;
			float[] tangent, impulse, impulseT;
			contact = contacts[i];
			
			// recalculate relative velocities with respect to contact point
			relativeA = vec2.sub(contact, A.getLinearMotion().getPosition());
			relativeB = vec2.sub(contact, B.getLinearMotion().getPosition());
			
			a = vec2.project(A.getLinearMotion().getVelocity(), vec2.normal(relativeA), A.getAngularMotion().getVelocity() );
			b = vec2.project(B.getLinearMotion().getVelocity(), vec2.normal(relativeB), B.getAngularMotion().getVelocity() );
			relativeV = vec2.sub(b,a);
			
			// get relative velocities. if separating, skip impulse
			float contactV = vec2.dot(relativeV, normal);
			if (contactV > 0) return;
			
			// get masses from objects to calculate the impulse strength
			float RAxN = vec2.cross(relativeA, normal)[2];
			float RBxN = vec2.cross(relativeB, normal)[2];
			float invMassSum = inv_massA + inv_massB + (RAxN*RAxN*A.inv_inertia) + (RBxN*RBxN*B.inv_inertia);
			
			// get impulse scaling ===============================
			
			float j = -(1.0f + restitution) * contactV;
			j /= invMassSum;
			j /= (float)contact_count;
			impulse = vec2.mul(normal, j);
			
			A.applyImpulse( vec2.neg(impulse), relativeA);
			B.applyImpulse( impulse, relativeB);
			
			//*
			
			// find friction impulse tangent =============================== 
			
			// recalculate relative velocities
			//a = Vector2.add(A.velocity, Vector2.normal(relativeA, A.angularVelocity) );
			//b = Vector2.add(B.velocity, Vector2.normal(relativeB, B.angularVelocity) );
			//relativeV = Vector2.sub(b,a);
			
			tangent = vec2.project(relativeV, normal, vec2.dot(relativeV, normal));
			tangent = vec2.normalize(tangent);
			
			float jt = -vec2.dot(relativeV, tangent);
			jt /= invMassSum;
			jt /= (float)contact_count;
			
			// if tangent impulse is practically 0, don't do anything 
			if (ImpulseMath.equal(jt, 0.0f)) return;
				
			// apply Coulumb's law for friction
			if (StrictMath.abs(jt) < j*sFriction)
			{
				impulseT = vec2.mul(tangent, jt);
			}
			else
			{
				impulseT = vec2.mul(tangent, -j*kFriction);
			}
			
			A.applyImpulse(vec2.neg(impulseT) ,relativeA);
			B.applyImpulse(impulseT, relativeB);
			//*/
		}
	}
	
	/**
	 * due to floating point precision errors, the object position may be off a little
	 */
	public void applyCorrection() 
	{
		boolean corrA = (!A.immovable);
		boolean corrB = (!B.immovable);
		
		float imassA = A.inv_mass;
		float imassB = B.inv_mass;
		
		float den = (imassA + imassB) * ImpulseMath.CORRECTION;
		if (den==0.0f) return;
		
		float num = FloatMath.max(penetration - ImpulseMath.ALLOWANCE, 0.0f);
		float corr = num/den;
		
		if (corrA) 
		{
			float[] p = vec2.project(A.getLinearMotion().getPosition(), normal, -corr*imassA );
			A.getLinearMotion().setPosition(p);
		}
		if (corrB) 
		{
			float[] p = vec2.project(B.getLinearMotion().getPosition(), normal,  corr*imassB );
			B.getLinearMotion().setPosition(p);
		}
	}
}
