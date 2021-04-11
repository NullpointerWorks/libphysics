package com.nullpointerworks.physics.engine.collision;

import com.nullpointerworks.physics.engine.Composite;
import com.nullpointerworks.physics.engine.Manifold;

public interface ICollisionSolver 
{
	public void solve( Manifold m, Composite A, Composite B );
}
