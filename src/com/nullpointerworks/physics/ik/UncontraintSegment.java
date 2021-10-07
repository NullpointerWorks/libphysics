package com.nullpointerworks.physics.ik;

import com.nullpointerworks.math.Approximate;
import com.nullpointerworks.math.vector.Vector2;

public class UncontraintSegment implements Segment
{
	protected static Vector2 V2 = new Vector2();
	
	private Segment parent;
	private float angle;
	private float length;
	private float[] base;
	
	public UncontraintSegment(float x, float y, float l)
	{
		base = V2.New(x,y);
		length = l;
		setParent(null);
	}
	
	public UncontraintSegment(float[] p, float l)
	{
		this(p[0], p[1], l);
	}
	
	public UncontraintSegment(Segment p, float l) 
	{
		this(p.getBase(), l);
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
		return tau + angle;
	}
	
	public void follow(float[] target)
	{
		float[] dir = V2.sub(target, base);
		angle = heading(dir);
		
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
	
	/*
	 * returns a value from -pi > x > pi
	 */
	private float heading(float[] delta)
	{
		return (float)Approximate.atan2(delta[1], delta[0]);
	}
}
