package com.nullpointerworks.physics.ik;

public class UncontraintSegment extends AbstractSegment
{
	public UncontraintSegment(float x, float y, float l)
	{
		setBase( V2.New(x,y) );
		setMagnitude(l);
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
	
	public void follow(float[] target)
	{
		float[] base = getBase();
		float[] dir = V2.sub(target, base);
		setAngle( atan2(dir) );
		
		dir = V2.normalize(dir);
		dir = V2.mul(dir, getMagnitude());
		base = V2.sub(target, dir);
		setBase(base);
		
		Segment parent = getParent();
		if (parent!=null) parent.follow(this);
	}
}
