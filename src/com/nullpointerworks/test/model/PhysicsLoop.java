package com.nullpointerworks.test.model;

import java.util.List;
import java.util.Vector;

import com.nullpointerworks.game.Asap;
import com.nullpointerworks.physics.engine.Composite;

public class PhysicsLoop extends Asap
{
	private List<Composite> bodies;
	
	public PhysicsLoop()
	{
		bodies = new Vector<Composite>();
	}
	
	public synchronized void addComposite(Composite c)
	{
		bodies.add(c);
	}
	
	@Override
	public void onInit() 
	{
		
	}

	@Override
	public void onUpdate(double time) 
	{
		
		
		
	}

	@Override
	public void onRender(double interpolation) 
	{
		
		
		
	}

	@Override
	public void onDispose() 
	{
		
	}
	
}
