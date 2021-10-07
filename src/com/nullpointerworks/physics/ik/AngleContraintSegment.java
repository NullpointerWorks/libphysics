package com.nullpointerworks.physics.ik;

import com.nullpointerworks.math.Approximate;
import com.nullpointerworks.math.vector.Vector2;

public class AngleContraintSegment implements Segment
{
	protected static Vector2 V2 = new Vector2();
	
	private Segment parent;
	private float angle;
	private float length;
	private float[] base;
	
	private float constraint;
	
	public AngleContraintSegment(float x, float y, float l, float phi)
	{
		base = V2.New(x,y);
		length = l;
		constraint = phi;
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
	
	// ===========================================================
	
	public void setParent(Segment p)
	{
		this.parent = p;
	}
	
	public Segment getParent()
	{
		return parent;
	}
	
	public float[] getBase()
	{
		return base;
	}
	
	public float[] getDest()
	{
		float[] r = V2.rotation(angle);
		return V2.project(base, r, length);
	}
	
	public float getAngle()
	{
		return angle;
	}
	
	public void follow(float[] target)
	{
		float[] dir = V2.sub(target, base);
		angle = heading(dir);
		
		float a = parent.getAngle() - angle;
		if (abs(a) > constraint)
		{
			float theta = parent.getAngle() - ( constraint * sign(a) );
			float[] rot = V2.rotation(theta);
			dir = V2.mul(rot, length);
		}
		
		dir = V2.normalize(dir);
		dir = V2.mul(dir, -length);
		base = V2.add(target, dir);
		
		if (parent!=null)
			parent.follow(this);
	}

	public void follow(Segment target)
	{
		follow( target.getBase() );
	}
	
	public void position(float[] delta)
	{
		base = V2.add(base,delta);
		if (parent!=null)
			parent.position(delta);
	}
	
	// ===========================================================

	private final float pi = 3.1415926f;
	private final float tau = 2f * pi;
	
	private float heading(float[] delta)
	{
		return tau + (float)Approximate.atan2(delta[1], delta[0]);
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
