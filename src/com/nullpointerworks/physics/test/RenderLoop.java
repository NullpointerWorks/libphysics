package com.nullpointerworks.physics.test;

import com.nullpointerworks.color.ColorFormat;
import com.nullpointerworks.color.Colorizer;
import com.nullpointerworks.core.Window;
import com.nullpointerworks.core.buffer.IntBuffer;
import com.nullpointerworks.game.Asap;
import com.nullpointerworks.physics.engine.Composite;

public class RenderLoop extends Asap
{
	private Window window;
	private IntBuffer canvas;
	private Colorizer color;
	private int BLACK;
	private int WHITE;
	
	private Composite immovableBox;
	
	public RenderLoop()
	{
		color = Colorizer.getColorizer( ColorFormat.RGB );
		BLACK = color.toInt(0,0,0);
		WHITE = color.toInt(255,255,255);
		
		canvas = new IntBuffer(800,600);
		window = new Window(800,600,"Rigid Body Physics Engine Test");
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
		canvas.clear(BLACK);
		
		window.swap( canvas.content() );
	}
	
	@Override
	public void onDispose() 
	{
		window.setVisible(false);
	}
}
