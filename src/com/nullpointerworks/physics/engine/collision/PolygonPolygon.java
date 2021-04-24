package com.nullpointerworks.physics.engine.collision;

import com.nullpointerworks.physics.engine.CollisionSolver;
import com.nullpointerworks.physics.engine.Composite;
import com.nullpointerworks.physics.engine.Manifold;
import com.nullpointerworks.physics.engine.math.ImpulseMath;
import com.nullpointerworks.physics.engine.math.MatrixMath;
import com.nullpointerworks.physics.engine.math.VectorMath;
import com.nullpointerworks.physics.engine.shape.Polygon;

public class PolygonPolygon implements CollisionSolver 
{
	/*
	 * Gilbert-Johnson-Keerthi (GJK) algorithm
	 */
	@Override
	public void solve(Manifold m, Composite A, Composite B)
	{
		m.contact_count = 0;
		
		// find face-to-face penetration axes
		int[] face_a = {0};
		float penetrationA = LeastPenetrationAxes(face_a, A,B);
		if (penetrationA >= 0.0f) return;
		
		int[] face_b = {0};
		float penetrationB = LeastPenetrationAxes(face_b, B,A);
		if (penetrationB >= 0.0f) return;
		
		// find the face of incidence
		if (ImpulseMath.gt(penetrationA, penetrationB))
		{
			solve(m,A,B, face_a[0],false);
		}
		else
		{
			solve(m,B,A, face_b[0],true);
		}
	}
	
	/*
	 * solve continuation
	 */
	public void solve(Manifold m, Composite R, Composite I, int reference_index, boolean handedness)
	{
		Polygon shapeR 	= (Polygon)R.getShape();
		
		float[][] incFace = new float[2][];
		IncidentFace(incFace, R, I, reference_index);
		
		float[][] refVertices	= shapeR.vertices;
		float[][] refMatrix 	= R.rotation;
		int l = refVertices.length;
		
		// get vertices from the reference polygon
		float[] v1 = refVertices[ reference_index ];
		float[] v2 = refVertices[(reference_index+1)%l ];
		
		// translate to world space
		v1 = MatrixMath.transform(refMatrix, v1);
		v2 = MatrixMath.transform(refMatrix, v2);
		v1 = VectorMath.add(v1, R.position);
		v2 = VectorMath.add(v2, R.position);
		
		// get normal and tangent of the world-space face
		float[] plane_normal 	= VectorMath.sub(v2, v1);
		plane_normal 			= VectorMath.normalize(plane_normal);
		float[] face_normal 	= VectorMath.normal(plane_normal, -1f);
		
		// find collision normal
		float refC 		= VectorMath.dot(face_normal, v1);
		float neg_side 	= -VectorMath.dot(plane_normal, v1);
		float pos_side 	= VectorMath.dot(plane_normal, v2);
		
		if ( clip(VectorMath.neg(plane_normal), neg_side, incFace ) < 2 ) 
			return;
		
		if ( clip(plane_normal, pos_side, incFace ) < 2 ) 
			return;
		
		// flip collision direction normal
		m.normal = face_normal;
		if (handedness)
		{
			m.normal = VectorMath.neg(m.normal);
		}
		
		// find separation
		int cp = 0;
		float separation1 = VectorMath.dot(face_normal, incFace[0]) - refC;
		float separation2 = VectorMath.dot(face_normal, incFace[1]) - refC;
		
		m.penetration = 0.0f;
		if (separation1 <= 0.0f)
		{
			m.contacts[cp] = VectorMath.copy(incFace[0]);
			m.penetration += -separation1;
			cp++;
		}
		
		if (separation2 <= 0.0f)
		{
			m.contacts[cp] = VectorMath.copy(incFace[1]);
			m.penetration += -separation2;
			cp++;
			
			m.penetration = m.penetration / (float)cp;
		}

		m.contact_count = cp;
	}
	
	// =========================================================================
	
	/*
	 * 
	 */
	private float LeastPenetrationAxes(int[] faceIndex, Composite A, Composite B)
	{
		Polygon shapeA 	= (Polygon)A.getShape();
		Polygon shapeB 	= (Polygon)B.getShape();
		
		float best_dist = -Float.MAX_VALUE;
		int best_index 	= 0;

		float[][] verticesA	= shapeA.vertices;
		float[][] verticesB	= shapeB.vertices;
		float[][] normals 	= shapeA.normals;
		
		float[][] rotationA = A.rotation;
		float[][] rotationB = B.rotation;
		float[][] trnsposeB = MatrixMath.transpose(rotationB);

		float[] posA = A.position;
		float[] posB = B.position;
		
		for (int i=0,l=verticesA.length; i<l; i++)
		{
			float[] vertex = verticesA[i];
			float[] normal = normals[i];
			
			// move normal to polygon B's model space
			float[] nw = MatrixMath.transform(rotationA, normal);
			float[] n = MatrixMath.transform(trnsposeB, nw);
			
			// move vertex from model space A to model space B
			float[] v = MatrixMath.transform(rotationA, vertex);
			v = VectorMath.add(v, posA);
			v = VectorMath.sub(v, posB);
			v = MatrixMath.transform(trnsposeB, v);
			
			// get support
			float[] s = support(verticesB, VectorMath.neg(n) );
			float dist = VectorMath.dot(n, VectorMath.sub(s, v) );
			if (dist > best_dist)
			{
				best_dist = dist;
				best_index = i;
			}
		}
		
		faceIndex[0] = best_index;
		return best_dist;
	}
	
	/**
	 * support function for Gilbert-Johnson-Keerthi (GJK) algorithm<br>
	 * www.toptal.com/game/video-game-physics-part-ii-collision-detection-for-solid-objects
	 */
	private float[] support(float[][] vertices, float[] dir) 
	{
		float bestProj = -Float.MAX_VALUE;
		float[] best_vertex = null;
		
		for (int i=0, l=vertices.length; i<l; i++)
		{
			float[] v = vertices[i];
			float proj = VectorMath.dot(v, dir);
			
			if (proj > bestProj)
			{
				best_vertex = v;
				bestProj = proj;
			}
		}
		return best_vertex;
	}
	
	/*
	 * 
	 */
	private void IncidentFace(float[][] v, Composite R, Composite I, int index)
	{
		Polygon refPoly = (Polygon)R.getShape();
		Polygon incPoly = (Polygon)I.getShape();
		
		float[][] refMatrix 	= R.rotation;
		float[] ref_normal 		= refPoly.normals[index];

		float[][] incMatrix 	= I.rotation;
		float[] incPosition 	= I.position;
		float[][] incTranspose 	= MatrixMath.transpose(incMatrix);
		
		// transform normal from model-space A to B
		ref_normal = MatrixMath.transform(refMatrix, ref_normal);
		ref_normal = MatrixMath.transform(incTranspose, ref_normal);
		
		// find most anti-normal face on incident polygon
		int incident_face = 0;
		float min_dot = Float.MAX_VALUE;
		
		float[][] vertices 	= incPoly.vertices;
		float[][] normals 	= incPoly.normals;
		
		int l = vertices.length;
		for (int i=0; i<l; i++)
		{
			float[] norm = normals[i];
			
			float dot = VectorMath.dot(ref_normal, norm);
			if (dot < min_dot)
			{
				min_dot = dot;
				incident_face = i;
			}
		}
		
		v[0] = MatrixMath.transform(incMatrix, vertices[incident_face] );
		v[0] = VectorMath.add(v[0], incPosition);
		
		incident_face = (incident_face+1)%l;
		
		v[1] = MatrixMath.transform(incMatrix, vertices[incident_face] );
		v[1] = VectorMath.add(v[1], incPosition);
	}
	
	/*
	 * 
	 */
	private int clip(float[] n, float c, float[][] face)
	{
		int sp = 0;
		float[][] out = 
		{
			VectorMath.copy(face[0]), 
			VectorMath.copy(face[1])
		};
		
		float d1 = VectorMath.dot(n, face[0]) - c;
		float d2 = VectorMath.dot(n, face[1]) - c;

		if (d1 <= 0.0f) out[sp++] = VectorMath.copy(face[0]);
		if (d2 <= 0.0f) out[sp++] = VectorMath.copy(face[1]);
		
		// xor, if one is negative
		if (d1*d2 < 0.0f)
		{
			float lambda = d1 / (d1 - d2);
			float[] A = VectorMath.sub(face[1], face[0]);
			A = VectorMath.mul(A, lambda);
			out[sp++] = VectorMath.add(A ,face[0]);
		}
		face[0] = out[0];
		face[1] = out[1];
		return sp;
	}
}
