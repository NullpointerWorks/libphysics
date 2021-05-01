package com.nullpointerworks.physics.engine.collision;

import static com.nullpointerworks.physics.engine.VectorMath.neg;

import com.nullpointerworks.physics.engine.Composite;
import com.nullpointerworks.physics.engine.manifold.Manifold;

public class PolygonCircle extends CirclePolygon
{
	@Override
	public void solve(Manifold m, Composite A, Composite B) 
	{
		super.solve(m, B, A);
		if (m.contact_count > 0)
		{
			m.normal = neg(m.normal);
		}
	}
}
