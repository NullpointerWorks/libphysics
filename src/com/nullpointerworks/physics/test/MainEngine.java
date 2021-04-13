package com.nullpointerworks.physics.test;

import com.nullpointerworks.core.Window;
import com.nullpointerworks.game.Asap;

public class MainEngine extends Asap
{
	public static void main(String[] args) 
	{
		new MainEngine();
	}
	
	private Window window;
	
	public MainEngine()
	{
		window = new Window(800,600,"Rigid Body Physics Engine Test");
		
		
		this.setTargetFPS(60);
		this.start();
	}

	@Override
	public void onInit() 
	{
		window.setVisible(true);
	}

	@Override
	public void onUpdate(double dt) 
	{
		
		
		
		
	}

	@Override
	public void onRender(double lerp) 
	{
		
		
		
		
	}

	@Override
	public void onDispose() 
	{
		window.setVisible(false);
		
	}

}
