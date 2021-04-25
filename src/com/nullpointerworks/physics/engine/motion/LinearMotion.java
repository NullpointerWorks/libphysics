package com.nullpointerworks.physics.engine.motion;

import static com.nullpointerworks.physics.engine.math.VectorMath.copy;
import static com.nullpointerworks.physics.engine.math.VectorMath.create;

public class LinearMotion 
{
	private float[] position;
	private float[] velocity;
	
	public LinearMotion()
	{
		setPosition( create(0f, 0f) );
		setVelocity( create(0f, 0f) );
	}
	
	public LinearMotion(float[] p, float[] v)
	{
		setPosition(p);
		setVelocity(v);
	}
	
	public void setPosition(float[] p)
	{
		position = copy(p);
	}
	
	public void setVelocity(float[] v)
	{
		velocity = copy(v);
	}
	
	public float[] getPosition()
	{
		return copy(position);
	}
	
	public float[] getVelocity()
	{
		return copy(velocity);
	}
	
	public LinearMotion getClone()
	{
		LinearMotion lm = new LinearMotion();
		lm.setPosition(position);
		lm.setVelocity(velocity);
		return lm;
	}
}
