package com.nullpointerworks.physics.engine.collision;

import com.nullpointerworks.physics.engine.CollisionSolver;
import com.nullpointerworks.physics.engine.Composite;
import com.nullpointerworks.physics.engine.Manifold;
import com.nullpointerworks.physics.engine.VectorMath;
import com.nullpointerworks.physics.engine.shape.Circle;

public class CircleCircle implements CollisionSolver 
{
	/*
	 * testing circle A onto B
	 */
	@Override
	public void solve(Manifold m, Composite A, Composite B) 
	{
		float radiusA = ((Circle)A.getShape()).getRadius();
		float radiusB = ((Circle)B.getShape()).getRadius();

		float[] posA = A.getLinearMotion().getPosition();
		float[] posB = B.getLinearMotion().getPosition();
		
		// find distances between points
		float[] tangent = VectorMath.sub(posB, posA);
		float dist 		= VectorMath.magnitude(tangent);
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
			m.contacts[0] 	= A.getLinearMotion().getPosition();
			m.normal 		= VectorMath.create(1f, 0f);
			m.penetration 	= radiusA;
		}
		// if partially overlapping
		else
		{
			m.contact_count = 1;
			m.penetration 	= radii - dist;
			m.normal 		= VectorMath.mul(tangent, 1f/dist); // normalize tangent
			m.contacts[0] 	= VectorMath.project(posA, m.normal, radiusA); // find deepest penetration
		}
	}
}
