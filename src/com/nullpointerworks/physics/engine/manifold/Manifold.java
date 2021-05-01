package com.nullpointerworks.physics.engine.manifold;

import static com.nullpointerworks.physics.engine.VectorMath.create;
import static com.nullpointerworks.physics.engine.VectorMath.normal;
import static com.nullpointerworks.physics.engine.VectorMath.normalize;
import static com.nullpointerworks.physics.engine.VectorMath.sub;

import java.util.ArrayList;
import java.util.List;

import static com.nullpointerworks.physics.engine.VectorMath.dot;
import static com.nullpointerworks.physics.engine.VectorMath.mul;
import static com.nullpointerworks.physics.engine.VectorMath.neg;
import static com.nullpointerworks.physics.engine.VectorMath.cross;
import static com.nullpointerworks.physics.engine.VectorMath.project;

import com.nullpointerworks.math.FloatMath;
import com.nullpointerworks.physics.engine.Composite;
import com.nullpointerworks.physics.engine.ImpulseMath;
import com.nullpointerworks.physics.engine.material.Material;

public class Manifold 
{
	private final Composite A, B;
	private final float resting;
	
	private List<ContactPoint> contactpoints;
	
	public float restitution;
	public float sFriction;
	public float kFriction;
	
	public float penetration;
	public int contact_count;
	public float[] normal;
	public float[][] contacts;
	
	public Manifold(Composite A, Composite B, float resting)
	{
		this.A = A; 
		this.B = B;
		this.resting = resting;
		contactpoints = new ArrayList<ContactPoint>();
		
		normal 		= new float[] {0f,0f};
		contacts 	= new float[][] {{0f,0f}, {0f,0f}};
	}
	
	public void addContactPoint(ContactPoint cp)
	{
		contactpoints.add(cp);
	}
	
	/**
	 * Returns true if the manifold has detected one or more collisions between the two objects, false otherwise.
	 * @return true if the manifold has detected one or more collisions between the two objects, false otherwise
	 */
	public boolean hasContacts()
	{
		return contactpoints.size() > 0;
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
		for (float[] contact : contacts) 
		{
			float[] Apos = A.getLinearMotion().getPosition();
			float[] Avel = A.getLinearMotion().getVelocity();
			float Aavel = A.getAngularMotion().getVelocity();
			
			float[] Bpos = B.getLinearMotion().getPosition();
			float[] Bvel = B.getLinearMotion().getVelocity();
			float Bavel = B.getAngularMotion().getVelocity();
			
			float[] relativeA = sub(contact, Apos);
			float[] relativeB = sub(contact, Bpos);
			
			float[] relativeV = project(Bvel, normal(relativeB), Bavel );
			relativeV = sub(relativeV, Avel);
			relativeV = project(relativeV, normal(relativeA), -Aavel );
			
			float m = dot(relativeV,relativeV); // square distance
			if (m < resting) 
			{
				restitution = 0.0f;
				break;
			}
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
		for (float[] contact : contacts)
		{
			float[] posA = A.getLinearMotion().getPosition();
			float[] velA = A.getLinearMotion().getVelocity();
			float avalA = A.getAngularMotion().getVelocity();
			
			float[] posB = B.getLinearMotion().getPosition();
			float[] velB = B.getLinearMotion().getVelocity();
			float avalB = B.getAngularMotion().getVelocity();
			
			// recalculate relative velocities with respect to contact point
			float[] relativeA = sub(contact, posA);
			float[] relativeB = sub(contact, posB);
			float[] a = project(velA, normal(relativeA), avalA );
			float[] b = project(velB, normal(relativeB), avalB );
			float[] relativeV = sub(b,a);
			
			// get relative velocities. if separating, skip impulse
			float contactV = dot(relativeV, normal);
			if (contactV > 0) return;
			
			// get masses from objects to calculate the impulse strength
			float RAxN = cross(relativeA, normal);
			float RBxN = cross(relativeB, normal);
			float invMassSum = inv_massA + inv_massB + (RAxN*RAxN*A.inv_inertia) + (RBxN*RBxN*B.inv_inertia);
			
			// get impulse scaling ===============================
			float j = -(1.0f + restitution) * contactV;
			j /= (invMassSum * (float)contact_count);
			float[] impulse = mul(normal, j);
			A.applyImpulse( neg(impulse), relativeA);
			B.applyImpulse( impulse, relativeB);
			
			// find friction impulse tangent =============================== 
			float[] tangent, impulseT;
			
			// recalculate relative velocities
			//a = Vector2.add(A.velocity, Vector2.normal(relativeA, A.angularVelocity) );
			//b = Vector2.add(B.velocity, Vector2.normal(relativeB, B.angularVelocity) );
			//relativeV = Vector2.sub(b,a);
			
			tangent = project(relativeV, normal, dot(relativeV, normal));
			tangent = normalize(tangent);
			
			float jt = -dot(relativeV, tangent);
			jt /= (invMassSum * (float)contact_count);
			
			// if tangent impulse is practically 0, don't do anything 
			if (ImpulseMath.equal(jt, 0.0f)) continue;
				
			// apply Coulumb's law for friction
			if (StrictMath.abs(jt) < j*sFriction)
			{
				impulseT = mul(tangent, jt);
			}
			else
			{
				impulseT = mul(tangent, -j*kFriction);
			}
			
			A.applyImpulse(neg(impulseT), relativeA);
			B.applyImpulse(impulseT, relativeB);
		}
	}
	
	/**
	 * due to floating point precision errors, the object position may be off a little
	 */
	public void applyCorrection() 
	{
		float imassA = A.inv_mass;
		float imassB = B.inv_mass;
		
		float den = (imassA + imassB) * ImpulseMath.CORRECTION;
		if (den==0.0f) return;
		
		float num = FloatMath.max(penetration - ImpulseMath.ALLOWANCE, 0.0f);
		float corr = num/den;
		
		if (!A.isImmovable()) 
		{
			float[] p = project(A.getLinearMotion().getPosition(), normal, -corr*imassA );
			A.getLinearMotion().setPosition(p);
		}
		if (!B.isImmovable()) 
		{
			float[] p = project(B.getLinearMotion().getPosition(), normal,  corr*imassB );
			B.getLinearMotion().setPosition(p);
		}
	}
}
