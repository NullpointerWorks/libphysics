package com.nullpointerworks.test.controller;

import static com.nullpointerworks.physics.engine.VectorMath.project;
import static com.nullpointerworks.physics.engine.VectorMath.add;
import static com.nullpointerworks.physics.engine.MatrixMath.rotation;
import static com.nullpointerworks.physics.engine.MatrixMath.transform;

import java.util.List;

import com.nullpointerworks.color.ColorFormat;
import com.nullpointerworks.color.Colorizer;
import com.nullpointerworks.core.buffer.IntBuffer;
import com.nullpointerworks.physics.engine.Composite;
import com.nullpointerworks.physics.engine.ShapeType;
import com.nullpointerworks.physics.engine.shape.Circle;
import com.nullpointerworks.physics.engine.shape.Polygon;
import com.nullpointerworks.test.model.PhysicsLoop;
import com.nullpointerworks.test.view.GameView;

public class CanvasRenderCommand implements RenderCommand
{
	private GameView vWindow; 
	private PhysicsLoop mPhysiscSim;
	
	private IntBuffer canvas;
	private Colorizer color;
	private int BLACK;
	private int WHITE;
	private int CYAN;
	private int RED;
	private float viewHeight;
	
	public CanvasRenderCommand(GameView v, PhysicsLoop s)
	{
		vWindow = v;
		mPhysiscSim = s;
		canvas = v.getCanvas();
		viewHeight = (float)canvas.getHeight();
		
		color 	= Colorizer.getColorizer( ColorFormat.RGB );
		BLACK 	= color.toInt(0,0,0);
		WHITE 	= color.toInt(255,255,255);
		CYAN 	= color.toInt(0,255,255);
		RED 	= color.toInt(255,0,0);
	}
	
	@Override
	public void onRender(double lerp) 
	{
		List<Composite> bodies = mPhysiscSim.getBodies();
		canvas.clear(BLACK);
		
		for (Composite c : bodies)
		{
			ShapeType st = c.getShape().getType();
			switch(st)
			{
			case Circle: drawCircle(c, canvas); break;
			case Polygon: drawPolygon(c, canvas); break;
			}
		}
		
		vWindow.swapCanvas();
	}
	
	private void drawCircle(Composite e, IntBuffer canvas)
	{
		Circle s 	= (Circle)e.getShape();
		float r 	= s.getRadius();
		float[] P 	= e.getLinearMotion().getPosition();
		float[] V 	= e.getLinearMotion().getVelocity();
		
		P[1] = viewHeight - P[1];
		V[1] = -V[1];
		
		circle(P, r, WHITE, canvas);
		
		//float a 	= e.orientation;
		//float[] J = project(P, rotation(a), r);
		//line(P, J, CYAN, canvas);
		
		float[] J = project(P, V, 1f);
		line(P, J, RED, canvas);
	}
	
	private void drawPolygon(Composite c, IntBuffer canvas)
	{
		Polygon poly = (Polygon)c.getShape();
		float[][] v = poly.getVertices();
		float[] p = c.getLinearMotion().getPosition();
		float[] V = c.getLinearMotion().getVelocity();
		float r = c.getAngularMotion().getOrientation();
		
		float[][] rMatrix = rotation(r);
		v = transform(rMatrix,v);
		
		float[] b1 = v[0];
		for (int i=1,l=v.length; i<=l; i++)
		{
			float[] b2 = v[i%l];
			float[] a = add(b1,p);
			float[] b = add(b2,p);
			
			a[1] = viewHeight - a[1];
			b[1] = viewHeight - b[1];
			
			line(a,b,WHITE,canvas);
			b1 = b2;
		}

		p[1] = viewHeight - p[1];
		V[1] = -V[1];
		float[] J = project(p, V, 1f);
		line(p, J, RED, canvas);
	}
	
	private void line(float[] p, float[] j, int c, IntBuffer s) 
	{
		int x1 = (int)(p[0]+0.5f);
		int y1 = (int)(p[1]+0.5f);
		int x2 = (int)(j[0]+0.5f);
		int y2 = (int)(j[1]+0.5f);
		line(x1, y1, x2, y2, c, s);
	}

	private void circle(float[] p, float fr, int c, IntBuffer s) 
	{
		int x = (int)(p[0]+0.5f);
		int y = (int)(p[1]+0.5f);
		int r = (int)(fr+0.5f);
		circle(x, y, r, c, s);
	}
	
	private void pixel(int x, int y, int w, int h, int c, IntBuffer screen)
	{
		if (x < 0) return;
		if (y < 0) return;
		if (x >= w) return;
		if (y >= h) return;
		screen.plot(x, y, c);
	}
	
	private void line(int x1,int y1,int x2,int y2, int c, IntBuffer s)
	{
		int w = s.getWidth();
		int h = s.getHeight();
		
		int dx = x2 - x1;
		int dy = y2 - y1;
		dx = (dx < 0)? -dx : dx;
		dy = (dy < 0)? -dy : dy;
		int sx = (x1 < x2)? 1 : -1;
		int sy = (y1 < y2)? 1 : -1;
		int err = dx - dy;
		int x = x1, y = y1;
		
		while (true) 
		{
			pixel(x, y, w, h, c, s);
			
		    if (x == x2)
		    if (y == y2)
		        break;
		    
		    int e2 = err << 1;
		    
		    if (e2 > -dy) 
		    {
		        err = err - dy;
		        x += sx;
		    }
		    
		    if (e2 < dx) 
		    {
		        err = err + dx;
		        y += sy;
		    }
		}
	}
	
	private void circle(int cx, int cy, int r, int c, IntBuffer s)
	{
		int w = s.getWidth();
		int h = s.getHeight();
		int x,y,xC,yC,radErr;
		
		x = r;
		y = 0;
		xC = 1-(r+r);
		yC = 1;
		radErr = 0;
		
		while(x >= y)
		{
			pixel(cx+x, cy+y, w, h, c, s);
			pixel(cx-x, cy+y, w, h, c, s);
			pixel(cx+x, cy-y, w, h, c, s);
			pixel(cx-x, cy-y, w, h, c, s);
			pixel(cx+y, cy+x, w, h, c, s);
			pixel(cx-y, cy+x, w, h, c, s);
			pixel(cx+y, cy-x, w, h, c, s);
			pixel(cx-y, cy-x, w, h, c, s);
			
			y++;
			radErr += yC;
			yC += 2;
			
			if ( (radErr+radErr+xC) > 0 )
			{
				x--;
				radErr += xC;
				xC += 2;
			}
		}
	}
}
