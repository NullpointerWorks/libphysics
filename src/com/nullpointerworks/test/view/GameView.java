package com.nullpointerworks.test.view;

import com.nullpointerworks.color.ColorFormat;
import com.nullpointerworks.color.Colorizer;
import com.nullpointerworks.core.Window;
import com.nullpointerworks.core.buffer.IntBuffer;

public class GameView 
{
	private Window window;
	private IntBuffer canvas;
	private Colorizer color;
	private int BLACK;
	private int WHITE;
	
	public GameView()
	{
		color = Colorizer.getColorizer( ColorFormat.RGB );
		BLACK = color.toInt(0,0,0);
		WHITE = color.toInt(255,255,255);
		
		canvas = new IntBuffer(800,600);
		window = new Window(800,600,"Rigid Body Physics Engine Test");
	}
	
	public void setVisible(boolean b)
	{
		window.setVisible(b);
	}
	
	public void clearCanvas()
	{
		canvas.clear(BLACK);
	}
	
	public void swapCanvas()
	{
		window.swap( canvas.content() );
	}
}
