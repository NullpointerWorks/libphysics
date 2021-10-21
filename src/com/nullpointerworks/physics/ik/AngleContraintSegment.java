package com.nullpointerworks.physics.ik;

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
		float angle = atan2(dir);
		setAngle( angle );
		
		// constraint angle
		float a = getAngle();
		float pa = parent.getAngle();
		
		if (!sameSign(a,pa))
		{
			if (pa > a) a += TAU;
		}
		
		float dt = a - pa;
		if (abs(dt) > constraint)
		{
			dt = parent.getAngle() + ( constraint * sign(dt) );
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
}
