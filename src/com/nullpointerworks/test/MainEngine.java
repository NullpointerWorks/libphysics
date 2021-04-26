package com.nullpointerworks.test;

import com.nullpointerworks.physics.PlanetaryGravitation;
import com.nullpointerworks.physics.engine.Composite;
import com.nullpointerworks.physics.engine.VectorMath;
import com.nullpointerworks.physics.engine.material.HeavyMaterial;
import com.nullpointerworks.physics.engine.material.LightMaterial;
import com.nullpointerworks.physics.engine.material.Material;
import com.nullpointerworks.physics.engine.material.StaticMaterial;
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
		mPhysiscSim.setGravity( VectorMath.create(0f, -10f) , (float)PlanetaryGravitation.EARTH );
		
		mGameSim.setTargetFPS(30);
		mGameSim.addRenderCommand(cRenderToScreen);
		
		mPhysiscSim.start();
		mGameSim.start();
		vWindow.setVisible(true);
		
		

		makeBox(mPhysiscSim, 50f, 200f, new HeavyMaterial(), false);
		makeBox(mPhysiscSim, 50f, 0f, new StaticMaterial(), true);
		
		
		makeCirle(mPhysiscSim, 20f, 500f, 500f, new LightMaterial(), false);
		makeCirle(mPhysiscSim, 100f, 250f, 100f, new StaticMaterial(), true);
		makeCirle(mPhysiscSim, 100f, 450f, 100f, new StaticMaterial(), true);
		
	}
	
	private void makeCirle(PhysicsLoop sim, float size, float x, float y, Material mat, boolean immovable) 
	{
		Composite circle = new Composite();
		circle.setMaterial( mat );
		circle.setShape( new Circle( size ) );
		circle.getLinearMotion().setPosition( new float[] {x, y} );
		
		circle.setImmovable(immovable);
		sim.addComposite(circle);
	}
	
	private void makeBox(PhysicsLoop sim, float x, float y, Material mat, boolean immovable) 
	{
		
		float[][] vBox = 
		{
			{-100f, 20f},
			{ 100f, 20f},
			{ 100f,-20f},
			{-100f,-20f}
		};
		
		float[][] nBox = 
		{
			{ 1f, 0f},
			{ 0f,-1f},
			{-1f, 0f},
			{ 0f,1f}	
		};
		
		Composite immovableBox = new Composite();
		immovableBox.setShape( new Polygon(vBox, nBox) );
		immovableBox.setMaterial( mat );
		immovableBox.getLinearMotion().setPosition( new float[] {x,y} );
		immovableBox.getAngularMotion().setOrientation( 3.1415f * 0.04f);
		immovableBox.setImmovable(immovable);
		
		sim.addComposite(immovableBox);
	}
}
