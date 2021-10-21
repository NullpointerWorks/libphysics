package com.nullpointerworks.physics.ik;

import com.nullpointerworks.math.Approximate;
import com.nullpointerworks.math.vector.Vector2;

public abstract class AbstractSegment implements Segment
{
	protected static Vector2 V2 = new Vector2();
	
	private Segment parent;
	private float angle;
	private float length;
	private float[] base;
	
	// ===========================================================
	
	@Override
	public void setParent(Segment p)
	{
		this.parent = p;
	}
	
	@Override
	public void setBase(float[] b)
	{
		base = b;
	}
	
	@Override
	public void setMagnitude(float l)
	{
		length = l;
	}
	
	@Override
	public void setAngle(float a)
	{
		angle = a;
	}
	
	// ===========================================================
	
	@Override
	public Segment getParent()
	{
		return parent;
	}
	
	@Override
	public float[] getBase()
	{
		return base;
	}
	
	@Override
	public float[] getDest()
	{
		float[] r = V2.rotation(angle);
		return V2.project(base, r, length);
	}
	
	@Override
	public float getMagnitude()
	{
		return length;
	}
	
	@Override
	public float getAngle()
	{
		return angle;
	}
	
	@Override
	public abstract void follow(float[] target);
	
	@Override
	public void follow(Segment target)
	{
		follow( target.getBase() );
	}
	
	@Override
	public void position(float[] delta)
	{
		base = V2.add(base,delta);
		if (parent!=null) parent.position(delta);
	}
	
	// ===========================================================
	
	protected final float PI = 3.1415926f;
	protected final float TAU = 2f * PI;
	
	/*
	 * atan2 returns a value from -pi > x > pi
	 */
	protected float atan2(float[] delta)
	{
		return (float)Approximate.atan2(delta[1], delta[0]);
	}
	
	protected float abs(float a) 
	{
		return (a<0f)? -a : a;
	}
	
	protected float sign(float a) 
	{
		return (a<0f)? -1f : 1f;
	}
	
	protected boolean sameSign(float a, float b) 
	{
		return (a * b) >= 0f;
	}
}
