package com.nullpointerworks.physics.engine.collision;

import com.nullpointerworks.physics.engine.Composite;
import com.nullpointerworks.physics.engine.Manifold;
import com.nullpointerworks.physics.engine.math.VectorMath;

public class PolygonCircle implements ICollisionSolver
{
	public static final PolygonCircle instance = new PolygonCircle();
	
	@Override
	public void solve(Manifold m, Composite A, Composite B) 
	{
		CirclePolygon.instance.solve(m, B, A);
		if (m.contact_count > 0)
		{
			m.normal = VectorMath.neg(m.normal);
		}
	}
}
