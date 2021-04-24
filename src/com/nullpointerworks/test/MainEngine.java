package com.nullpointerworks.test;

import com.nullpointerworks.physics.engine.Composite;
import com.nullpointerworks.physics.engine.Material;
import com.nullpointerworks.physics.engine.shape.Polygon;
import com.nullpointerworks.test.controller.CanvasRenderCommand;
import com.nullpointerworks.test.controller.RenderCommand;
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
		PhysicsLoop mPhysiscSim 		= new PhysicsLoop();
		GameLoop mGameSim 				= new GameLoop();
		
		GameView vWindow 				= new GameView();
		
		RenderCommand cRenderToScreen 	= new CanvasRenderCommand(vWindow, mPhysiscSim);
		
		
		
		
		mPhysiscSim.setTargetFPS(80);
		mGameSim.setTargetFPS(60);
		mGameSim.addRenderCommand(cRenderToScreen);
		
		mPhysiscSim.start();
		mGameSim.start();
		vWindow.setVisible(true);
	}
	
	private void makeBox() 
	{
		
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
		
	}
}
