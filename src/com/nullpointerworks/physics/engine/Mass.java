package com.nullpointerworks.physics.engine;

public class Mass 
{
	private float mass;
	private float inv_mass;
	private float inertia;
	private float inv_inertia;
	
	public Mass()
	{
		setMass(0f);
		setInertia(0f);
	}
	
	private Mass(float m, float i)
	{
		setMass(m);
		setInertia(i);
	}
	
	public void setMass(float m)
	{
		mass = m;
		inv_mass = (m==0.0f)? Float.MAX_VALUE: 1f/m;
	}
	
	public void setInertia(float i)
	{
		inertia = i;
		inv_inertia = (i==0.0f)? Float.MAX_VALUE: 1f/i;
	}

	/**
	 * calculate the mass<br>
	 * d = density<br>
	 * V = volume<br>
	 * <pre>
	 * mass = density * volume
	 * </pre>
	 */
	public float getMass()
	{
		return mass;
	}
	
	public float getInverseMass()
	{
		return inv_mass;
	}
	
	/**
	 * set the moment of inertia. This value comes from the Shape class.
	 * <pre>
	 * i = SUM(m*r^2)
	 * </pre>
	 */
	public float getInertia()
	{
		return inertia;
	}
	
	public float getInverseInertia()
	{
		return inv_inertia;
	}
	
	public Mass getClone()
	{
		return new Mass(mass,inertia);
	}
}
