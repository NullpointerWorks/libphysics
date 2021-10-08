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
		Segment parent = getParent();
		float l = getMagnitude();
		float[] base = getBase();
		float[] dir = V2.sub(target, base);
		setAngle( heading(dir) );
		
		dir = V2.normalize(dir);
		dir = V2.mul(dir, -l);
		base = V2.add(target, dir);
		setBase(base);
		
		if (parent!=null) parent.follow(this);
	}
}
