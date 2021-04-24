package com.nullpointerworks.test;

import com.nullpointerworks.physics.engine.Composite;
import com.nullpointerworks.physics.engine.Material;
import com.nullpointerworks.physics.engine.shape.Polygon;
import com.nullpointerworks.test.model.GameLoop;
import com.nullpointerworks.test.model.PhysicsLoop;
import com.nullpointerworks.test.view.GameView;

public class MainEngine
{
	public static void main(String[] args) 
	{
		new MainEngine();
	}
	
	public MainEngine()
	{
		GameView vWindow = new GameView();
		
		PhysicsLoop mPhysiscSim = new PhysicsLoop();
		mPhysiscSim.setTargetFPS(80);
		
		GameLoop mGameSim = new GameLoop();
		mGameSim.setTargetFPS(60);
		
		
		
		
		
		float[][] vBox = 
		{
			{  0f,  0f},
			{100f,  0f},
			{100f, 20f},
			{  0f, 20f}	
		};
		
		float[][] nBox = 
		{
			{ 1f, 0f},
			{ 0f, 1f},
			{-1f, 0f},
			{ 0f,-1f}	
		};
		
		Composite immovableBox = new Composite();
		immovableBox.setShape( new Polygon(vBox, nBox) );
		immovableBox.setMaterial( Material.Static() );
		
		
		
		
		
		mPhysiscSim.start();
		mGameSim.start();
	}
}
