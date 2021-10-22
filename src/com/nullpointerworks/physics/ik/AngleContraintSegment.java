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
		setAngle( atan2(dir) );
		
		// get angles to compare
		float a = getAngle();
		float pa = parent.getAngle();
		
		// correct for the sin(x) plane traversal (difference between positive and negative angles)
		if (!sameSign(a,pa)) 
		{
			if (pa > a) pa += TAU; // bending down
			else
			if (a > pa) a += TAU; // bending up
			
			if (dir[0] < 0f) // if directing to the left, swap calculation
			{
				float t = a;
				a = pa;
				pa = t;
			}
		}
		float dt = a - pa; // find difference in angle 
		
		// constraint angle
		if (abs(dt) > constraint)
		{
			dt = parent.getAngle() + ( constraint * sign(dt) );
			float[] rot = V2.rotation(dt);
			dir = V2.mul(rot, l);
		}
		
		// re-calc direction
		dir = V2.normalize(dir);
		dir = V2.mul(dir, l);
		base = V2.sub(target, dir);
		setBase(base);
		
		// call parent
		if (parent!=null) parent.follow(this);
	}
}
