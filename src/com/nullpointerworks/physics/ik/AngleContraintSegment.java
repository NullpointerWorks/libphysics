package com.nullpointerworks.physics.ik;

import com.nullpointerworks.math.Approximate;

public class AngleContraintSegment extends AbstractSegment
{
	private float constraint;
	
	public AngleContraintSegment(float x, float y, float l, float phi)
	{
		setBase( V2.New(x,y) );
		setMagnitude(l);
		setConstraint(phi);
		setParent(null);
	}
	
	public AngleContraintSegment(float[] p, float l, float phi)
	{
		this(p[0], p[1], l, phi);
	}
	
	public AngleContraintSegment(Segment p, float l, float phi) 
	{
		this(p.getBase(), l, phi);
		setParent(p);
	}
	
	public void setConstraint(float phi)
	{
		constraint = phi;
	}
	
	// ===========================================================
	
	public void follow(float[] target)
	{
		// get some info
		Segment parent = getParent();
		float l = getMagnitude();
		float[] base = getBase();
		
		// calc current angle
		float[] dir = V2.sub(target, base);
		float angle = heading(dir);
		setAngle( angle );
		
		// constraint angle
		float dt = parent.getAngle() - angle;
		if (abs(dt) > constraint)
		{
			dt = parent.getAngle() - ( constraint * sign(dt) );
			float[] rot = V2.rotation(dt);
			dir = V2.mul(rot, l);
		}
		
		// re-calc direction
		dir = V2.normalize(dir);
		dir = V2.mul(dir, -l);
		base = V2.add(target, dir);
		setBase(base);
		
		// call parent
		if (parent!=null) parent.follow(this);
	}
	
	// ===========================================================
	
	private final float PI = 3.1415926f;
	private final float TAU = 2f * PI;
	
	/*
	 * atan2 returns a value from -pi > x > pi
	 */
	private float heading(float[] delta)
	{
		return (float)Approximate.atan2(delta[1], delta[0]);
	}
	
	private float abs(float a) 
	{
		return (a<0f)? -a : a;
	}
	
	private float sign(float a) 
	{
		return (a<0f)? -1f : 1f;
	}
}
