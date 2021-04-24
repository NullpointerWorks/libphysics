package com.nullpointerworks.physics.test;

import com.nullpointerworks.physics.engine.Composite;
import com.nullpointerworks.physics.engine.Material;
import com.nullpointerworks.physics.engine.shape.Polygon;

public class MainEngine
{
	public static void main(String[] args) 
	{
		new MainEngine();
	}
	
	public MainEngine()
	{
		PhysicsLoop pl = new PhysicsLoop();
		pl.setTargetFPS(80);
		pl.start();
		
		RenderLoop rl = new RenderLoop();
		rl.setTargetFPS(60);
		rl.start();
		
		
		
		
		

		float[][] vBox = 
		{
			{0f, 0f},
			{100f, 0f},
			{100f, 20f},
			{0f, 20f}	
		};
		
		float[][] nBox = 
		{
			{1f,0f},
			{0f,1f},
			{-1f,0f},
			{0f,-1f}	
		};
		
		Composite immovableBox = new Composite();
		immovableBox.setShape( new Polygon(vBox, nBox) );
		immovableBox.setMaterial( Material.Static() );
		
		
		pl.addComposite(immovableBox);
		
		
		
		
		
		
		
		
	}
}
