package com.nullpointerworks.test.view;

import com.nullpointerworks.core.Window;
import com.nullpointerworks.core.buffer.IntBuffer;

public class GameView 
{
	private Window window;
	private IntBuffer canvas;
	
	public GameView()
	{
		canvas = new IntBuffer(800,600);
		window = new Window(800,600,"Rigid Body Physics Engine Test");
	}
	
	public void setVisible(boolean b)
	{
		window.setVisible(b);
	}
	
	public void setCanvasColor(int c)
	{
		canvas.clear(c);
	}
	
	public void setPixel(int x, int y, int c)
	{
		canvas.plot(x,y,c);
	}
	
	public void swapCanvas()
	{
		window.swap( canvas.content() );
	}
}
