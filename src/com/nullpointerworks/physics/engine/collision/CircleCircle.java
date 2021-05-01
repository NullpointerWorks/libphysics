package com.nullpointerworks.physics.engine.collision;

import com.nullpointerworks.physics.engine.CollisionSolver;
import com.nullpointerworks.physics.engine.Composite;
import com.nullpointerworks.physics.engine.ImpulseMath;
import com.nullpointerworks.physics.engine.VectorMath;
import com.nullpointerworks.physics.engine.manifold.ContactPoint;
import com.nullpointerworks.physics.engine.manifold.Manifold;
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
		
		m.contact_count = 1;
		
		// if circles are exactly on top of each other
		if (ImpulseMath.equal(dist,0f))
		{
			float[] P 	= A.getLinearMotion().getPosition();
			float[] N 	= VectorMath.create(1f, 0f);
			float pd 	= radiusA;
			ContactPoint cp = new ContactPoint(P, N, pd);
			m.addContactPoint(cp);
			
			
			
			m.contacts[0] 	= A.getLinearMotion().getPosition();
			m.normal 		= VectorMath.create(1f, 0f); // drive the two circles away in some arbitrary direction.
			m.penetration 	= radiusA;
		}
		// if in contact
		else
		{
			float[] P 	= VectorMath.project(posA, m.normal, radiusA); // find deepest penetration
			float[] N 	= VectorMath.normalize(tangent); // normalize tangent
			float pd 	= radii - dist;
			ContactPoint cp = new ContactPoint(P, N, pd);
			m.addContactPoint(cp);
			
			
			
			m.contacts[0] 	= VectorMath.project(posA, m.normal, radiusA); // find deepest penetration
			m.normal 		= VectorMath.normalize(tangent); // normalize tangent
			m.penetration 	= radii - dist;
		}
	}
}
