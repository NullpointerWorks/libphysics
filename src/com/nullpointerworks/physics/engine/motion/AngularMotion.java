package com.nullpointerworks.physics.engine.motion;

public class AngularMotion 
{
	private float orientation;
	private float angularVelocity;
	
	public AngularMotion()
	{
		orientation = 0f;
		angularVelocity = 0f;
	}
	
	public AngularMotion(float o, float v)
	{
		setOrientation(o);
		setVelocity(v);
	}
	
	public void setOrientation(float p)
	{
		orientation = p;
	}
	
	public void setVelocity(float v)
	{
		angularVelocity = v;
	}
	
	public float getOrientation()
	{
		return orientation;
	}
	
	public float getVelocity()
	{
		return angularVelocity;
	}
	
	public AngularMotion getClone()
	{
		AngularMotion m = new AngularMotion();
		m.setOrientation(orientation);
		m.setVelocity(angularVelocity);
		return m;
	}
}
