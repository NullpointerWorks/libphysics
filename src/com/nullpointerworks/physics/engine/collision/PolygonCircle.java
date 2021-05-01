package com.nullpointerworks.physics.engine.collision;

import com.nullpointerworks.physics.engine.CollisionSolver;
import com.nullpointerworks.physics.engine.Composite;
import com.nullpointerworks.physics.engine.VectorMath;
import com.nullpointerworks.physics.engine.manifold.Manifold;

public class PolygonCircle implements CollisionSolver
{
	private CirclePolygon instance = new CirclePolygon();

	@Override
	public void solve(Manifold m, Composite A, Composite B) 
	{
		instance.solve(m, B, A);
		if (m.contact_count > 0)
		{
			m.normal = VectorMath.neg(m.normal);
		}
	}
}
