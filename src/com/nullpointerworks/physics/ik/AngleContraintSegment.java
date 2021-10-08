package com.nullpointerworks.physics.ik;

public class AngleContraintSegment extends AbstractSegment
{
	private float constraint;
	
	public AngleContraintSegment(float x, float y, float l, float phi)
	{
		setBase( V2.New(x,y) );
		setMagnitude(l);
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
		float a = parent.getAngle() - angle;
		if (abs(a) > constraint)
		{
			float theta = parent.getAngle() - ( constraint * sign(a) );
			float[] rot = V2.rotation(theta);
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
	
	private float abs(float a) 
	{
		return (a<0f)? -a : a;
	}
	
	private float sign(float a) 
	{
		return (a<0f)? -1f : 1f;
	}
}
