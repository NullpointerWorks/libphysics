package com.nullpointerworks.physics.engine.collision;

import com.nullpointerworks.physics.engine.Composite;
import com.nullpointerworks.physics.engine.Manifold;

import libMath.vector.Vector2;

public class CircleCircle implements ICollisionSolver 
{
	public static final CircleCircle instance = new CircleCircle();
	
	/*
	 * testing circle A onto B
	 */
	@Override
	public void solve(Manifold m, Composite A, Composite B) 
	{
		float radiusA = ((Circle)A.shape).radius;
		float radiusB = ((Circle)B.shape).radius;

		float[] posA = A.position;
		float[] posB = B.position;
		
		// find distances between points
		float[] tangent = Vector2.sub(posB, posA);
		float dist 		= Vector2.magnitude(tangent);
		float radii		= radiusA + radiusB;
		
		// no collision
		if (dist > radii)
		{
			m.contact_count = 0;
			return;
		}
		
		// if circles are exactly on top of each other
		if (dist == 0.0f)
		{
			// drive the two circles away in some direction.
			m.contact_count = 1;
			m.contacts[0] 	= Vector2.copy(A.position);
			m.normal 		= Vector2.New(1f, 0f);
			m.penetration 	= radiusA;
		}
		// if partially overlapping
		else
		{
			m.contact_count = 1;
			m.penetration 	= radii - dist;
			m.normal 		= Vector2.mul(tangent, 1f/dist); // normalize tangent
			m.contacts[0] 	= Vector2.projection(posA, m.normal, radiusA); // find deepest penetration
		}
	}
}
