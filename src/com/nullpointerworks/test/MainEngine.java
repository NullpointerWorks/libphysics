package com.nullpointerworks.test;

import com.nullpointerworks.physics.GravitationConstants;
import com.nullpointerworks.physics.engine.Composite;
import com.nullpointerworks.physics.engine.material.LightMaterial;
import com.nullpointerworks.physics.engine.material.StaticMaterial;
import com.nullpointerworks.physics.engine.math.VectorMath;
import com.nullpointerworks.physics.engine.shape.Circle;
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
		GameView vWindow 				= new GameView(800, 600);
		RenderCommand cRenderToScreen 	= new CanvasRenderCommand(vWindow, mPhysiscSim);
		
		mPhysiscSim.setTargetFPS(60);
		mPhysiscSim.setGravity( VectorMath.create(0f, -1f) , (float)GravitationConstants.EARTH );
		
		mGameSim.setTargetFPS(30);
		mGameSim.addRenderCommand(cRenderToScreen);
		
		mPhysiscSim.start();
		mGameSim.start();
		vWindow.setVisible(true);
		
		
		makeCirle(mPhysiscSim);
	}
	
	private void makeCirle(PhysicsLoop sim) 
	{
		Composite circle = new Composite();
		circle.setMaterial( new LightMaterial() );
		circle.setShape( new Circle( 20f ) );
		circle.getLinearMotion().setPosition( new float[] {300f,100f} );
		
		circle.setImmovable(true);
		sim.addComposite(circle);
	}
	
	private void makeBox(PhysicsLoop sim) 
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
		immovableBox.setMaterial( new StaticMaterial() );

		sim.addComposite(immovableBox);
	}
}
