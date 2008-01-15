/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

package Box2D.Collision.Shapes{



import Box2D.Common.Math.*;
import Box2D.Common.*
import Box2D.Collision.Shapes.*;
import Box2D.Dynamics.*
import Box2D.Collision.*

// A convex polygon. The position of the polygon (m_position) is the
// position of the centroid. The vertices of the incoming polygon are pre-rotated
// according to the local rotation. The vertices are also shifted to be centered
// on the centroid. Since the local rotation is absorbed into the vertex
// coordinates, the polygon rotation is equal to the body rotation. However,
// the polygon position is centered on the polygon centroid. This simplifies
// some collision algorithms.

public class b2PolyShape extends b2Shape
{
	public override function TestPoint(p:b2Vec2):Boolean{
		
		//var pLocal:b2Vec2 = b2Math.b2MulTMV(m_R, b2Math.SubtractVV(p, m_position));
		var pLocal:b2Vec2 = new b2Vec2();
		pLocal.SetV(p);
		pLocal.Subtract(m_position);
		pLocal.MulTM(m_R);
		
		for (var i:int = 0; i < m_vertexCount; ++i)
		{
			//var dot:Number = b2Math.b2Dot(m_normals[i], b2Math.SubtractVV(pLocal, m_vertices[i]));
			var tVec:b2Vec2 = new b2Vec2();
			tVec.SetV(pLocal);
			tVec.Subtract(m_vertices[i]);
			
			var dot:Number = b2Math.b2Dot(m_normals[i], tVec);
			if (dot > 0.0)
			{
				return false;
			}
		}
		
		return true;
	}
	
	//--------------- Internals Below -------------------
	// Temp vec for PolyCentroid
	static private var tempVec:b2Vec2 = new b2Vec2();
	static private var tAbsR:b2Mat22 = new b2Mat22()
	
	public function b2PolyShape(def:b2ShapeDef, body:b2Body, newOrigin:b2Vec2){
		
		super(def, body);
		
		var i:int;
		
		
		var hX:Number;
		var hY:Number;
		
		var tVec:b2Vec2;
		
		var aabb:b2AABB = new b2AABB();
		
		// Vertices
		m_vertices = new Array(b2Settings.b2_maxPolyVertices);
		m_coreVertices = new Array(b2Settings.b2_maxPolyVertices);
		//for (i = 0; i < b2Settings.b2_maxPolyVertices; i++)
		//	m_vertices[i] = new b2Vec2();
			
		// Normals
		m_normals = new Array(b2Settings.b2_maxPolyVertices);
		//for (i = 0; i < b2Settings.b2_maxPolyVertices; i++)
		//	m_normals[i] = new b2Vec2();
		
		//b2Settings.b2Assert(def.type == e_boxShape || def.type == e_polyShape);
		m_type = b2Shape.e_polyShape;
		
		var localR:b2Mat22 = new b2Mat22(def.localRotation);
		
		// Get the vertices transformed into the body frame.
		if (def.type == b2Shape.e_boxShape)
		{
			//m_localCentroid = def.localPosition - newOrigin;
			m_localCentroid.x = def.localPosition.x - newOrigin.x;
			m_localCentroid.y = def.localPosition.y - newOrigin.y;
			
			var box:b2BoxDef = def as b2BoxDef;
			m_vertexCount = 4;
			hX = box.extents.x;
			hY = box.extents.y;
			
			//hc.x = b2Max(0.0f, h.x - 2.0f * b2_linearSlop);
			var hcX:Number = Math.max(0.0, hX - 2.0 * b2Settings.b2_linearSlop);
			//hc.y = b2Max(0.0f, h.y - 2.0f * b2_linearSlop);
			var hcY:Number = Math.max(0.0, hY - 2.0 * b2Settings.b2_linearSlop);
			
			//m_vertices[0] = b2Mul(localR, b2Vec2(h.x, h.y));
			tVec = m_vertices[0] = new b2Vec2();
			tVec.x = localR.col1.x * hX + localR.col2.x * hY;
			tVec.y = localR.col1.y * hX + localR.col2.y * hY;
			//m_vertices[1] = b2Mul(localR, b2Vec2(-h.x, h.y));
			tVec = m_vertices[1] = new b2Vec2();
			tVec.x = localR.col1.x * -hX + localR.col2.x * hY;
			tVec.y = localR.col1.y * -hX + localR.col2.y * hY;
			//m_vertices[2] = b2Mul(localR, b2Vec2(-h.x, -h.y));
			tVec = m_vertices[2] = new b2Vec2();
			tVec.x = localR.col1.x * -hX + localR.col2.x * -hY;
			tVec.y = localR.col1.y * -hX + localR.col2.y * -hY;
			//m_vertices[3] = b2Mul(localR, b2Vec2(h.x, -h.y));
			tVec = m_vertices[3] = new b2Vec2();
			tVec.x = localR.col1.x * hX + localR.col2.x * -hY;
			tVec.y = localR.col1.y * hX + localR.col2.y * -hY;
			
			//m_coreVertices[0] = b2Mul(localR, b2Vec2(hc.x, hc.y));
			tVec = m_coreVertices[0] = new b2Vec2();
			tVec.x = localR.col1.x * hcX + localR.col2.x * hcY;
			tVec.y = localR.col1.y * hcX + localR.col2.y * hcY;
			//m_coreVertices[1] = b2Mul(localR, b2Vec2(-hc.x, hc.y));
			tVec = m_coreVertices[1] = new b2Vec2();
			tVec.x = localR.col1.x * -hcX + localR.col2.x * hcY;
			tVec.y = localR.col1.y * -hcX + localR.col2.y * hcY;
			//m_coreVertices[2] = b2Mul(localR, b2Vec2(-hc.x, -hc.y));
			tVec = m_coreVertices[2] = new b2Vec2();
			tVec.x = localR.col1.x * -hcX + localR.col2.x * -hcY;
			tVec.y = localR.col1.y * -hcX + localR.col2.y * -hcY;
			//m_coreVertices[3] = b2Mul(localR, b2Vec2(hc.x, -hc.y));
			tVec = m_coreVertices[3] = new b2Vec2();
			tVec.x = localR.col1.x * hcX + localR.col2.x * -hcY;
			tVec.y = localR.col1.y * hcX + localR.col2.y * -hcY;
		}
		else
		{
			var poly:b2PolyDef = def as b2PolyDef;
			
			m_vertexCount = poly.vertexCount;
			//b2Settings.b2Assert(3 <= m_vertexCount && m_vertexCount <= b2Settings.b2_maxPolyVertices);
			//b2Vec2 centroid = PolyCentroid(poly->vertices, poly->vertexCount);
			PolyCentroid(poly.vertices, poly.vertexCount, tempVec);
			var centroidX:Number = tempVec.x;
			var centroidY:Number = tempVec.y;
			//m_localCentroid = def->localPosition + b2Mul(localR, centroid) - newOrigin;
			m_localCentroid.x = def.localPosition.x + (localR.col1.x * centroidX + localR.col2.x * centroidY) - newOrigin.x;
			m_localCentroid.y = def.localPosition.y + (localR.col1.y * centroidX + localR.col2.y * centroidY) - newOrigin.y;
			
			for (i = 0; i < m_vertexCount; ++i)
			{
				m_vertices[i] = new b2Vec2();
				m_coreVertices[i] = new b2Vec2();
				
				//m_vertices[i] = b2Mul(localR, poly->vertices[i] - centroid);
				hX = poly.vertices[i].x - centroidX;
				hY = poly.vertices[i].y - centroidY;
				m_vertices[i].x = localR.col1.x * hX + localR.col2.x * hY;
				m_vertices[i].y = localR.col1.y * hX + localR.col2.y * hY;
				
				//b2Vec2 u = m_vertices[i];
				var uX:Number = m_vertices[i].x;
				var uY:Number = m_vertices[i].y;
				//float32 length = u.Length();
				var length:Number = Math.sqrt(uX*uX + uY*uY);
				if (length > Number.MIN_VALUE)
				{
					uX *= 1.0 / length;
					uY *= 1.0 / length;
				}
				
				//m_coreVertices[i] = m_vertices[i] - 2.0f * b2_linearSlop * u;
				m_coreVertices[i].x = m_vertices[i].x - 2.0 * b2Settings.b2_linearSlop * uX;
				m_coreVertices[i].y = m_vertices[i].y - 2.0 * b2Settings.b2_linearSlop * uY;
			}
			
		}
			
		// Compute bounding box. TODO_ERIN optimize OBB
		//var minVertex:b2Vec2 = new b2Vec2(Number.MAX_VALUE, Number.MAX_VALUE);
		var minVertexX:Number = Number.MAX_VALUE;
		var minVertexY:Number = Number.MAX_VALUE;
		var maxVertexX:Number = -Number.MAX_VALUE;
		var maxVertexY:Number = -Number.MAX_VALUE;
		m_maxRadius = 0.0;
		for (i = 0; i < m_vertexCount; ++i)
		{
			var v:b2Vec2 = m_vertices[i];
			//minVertex = b2Math.b2MinV(minVertex, m_vertices[i]);
			minVertexX = Math.min(minVertexX, v.x);
			minVertexY = Math.min(minVertexY, v.y);
			//maxVertex = b2Math.b2MaxV(maxVertex, m_vertices[i]);
			maxVertexX = Math.max(maxVertexX, v.x);
			maxVertexY = Math.max(maxVertexY, v.y);
			//m_maxRadius = b2Max(m_maxRadius, v.Length());
			m_maxRadius = Math.max(m_maxRadius, v.Length());
		}
		
		m_localOBB.R.SetIdentity();
		//m_localOBB.center = 0.5 * (minVertex + maxVertex);
		m_localOBB.center.Set((minVertexX + maxVertexX) * 0.5, (minVertexY + maxVertexY) * 0.5);
		//m_localOBB.extents = 0.5 * (maxVertex - minVertex);
		m_localOBB.extents.Set((maxVertexX - minVertexX) * 0.5, (maxVertexY - minVertexY) * 0.5);
		
		// Compute the edge normals and next index map.
		var i1:int;
		var i2:int;
		for (i = 0; i < m_vertexCount; ++i)
		{
			m_normals[i] =  new b2Vec2();
			i1 = i;
			i2 = i + 1 < m_vertexCount ? i + 1 : 0;
			//b2Vec2 edge = m_vertices[i2] - m_vertices[i1];
			//var edgeX:Number = m_vertices[i2].x - m_vertices[i1].x;
			//var edgeY:Number = m_vertices[i2].y - m_vertices[i1].y;
			//m_normals[i] = b2Cross(edge, 1.0f);
			m_normals[i].x = m_vertices[i2].y - m_vertices[i1].y;//1.0 * edgeY;
			m_normals[i].y = -(m_vertices[i2].x - m_vertices[i1].x);//-1.0 * edgeX;
			m_normals[i].Normalize();
		}
		
		// Ensure the polygon in convex. TODO_ERIN compute convex hull.
		for (i = 0; i < m_vertexCount; ++i)
		{
			i1 = i;
			i2 = i + 1 < m_vertexCount ? i + 1 : 0;
			
			//b2Settings.b2Assert(b2Math.b2CrossVV(m_normals[i1], m_normals[i2]) > Number.MIN_VALUE);
		}
		
		m_R.SetM(m_body.m_R);
		//m_position.SetV( m_body.m_position  + b2Mul(m_body->m_R, m_localCentroid) );
		m_position.x = m_body.m_position.x + (m_R.col1.x * m_localCentroid.x + m_R.col2.x * m_localCentroid.y);
		m_position.y = m_body.m_position.y + (m_R.col1.y * m_localCentroid.x + m_R.col2.y * m_localCentroid.y);
		
		//var R:b2Mat22 = b2Math.b2MulMM(m_R, m_localOBB.R);
			//R.col1 = b2MulMV(m_R, m_localOBB.R.col1); 
			tAbsR.col1.x = m_R.col1.x * m_localOBB.R.col1.x + m_R.col2.x * m_localOBB.R.col1.y; 
			tAbsR.col1.y = m_R.col1.y * m_localOBB.R.col1.x + m_R.col2.y * m_localOBB.R.col1.y; 
			//R.col2 = b2MulMV(m_R, m_localOBB.R.col2)
			tAbsR.col2.x = m_R.col1.x * m_localOBB.R.col2.x + m_R.col2.x * m_localOBB.R.col2.y; 
			tAbsR.col2.y = m_R.col1.y * m_localOBB.R.col2.x + m_R.col2.y * m_localOBB.R.col2.y; 
		//var absR:b2Mat22 = b2Math.b2AbsM(R);
		tAbsR.Abs()
		
		//h = b2Math.b2MulMV(tAbsR, m_localOBB.extents);
		hX = tAbsR.col1.x * m_localOBB.extents.x + tAbsR.col2.x * m_localOBB.extents.y;
		hY = tAbsR.col1.y * m_localOBB.extents.x + tAbsR.col2.y * m_localOBB.extents.y;
		
		//var position:b2Vec2 = m_position + b2Mul(m_R, m_localOBB.center);
		var positionX:Number = m_position.x + (m_R.col1.x * m_localOBB.center.x + m_R.col2.x * m_localOBB.center.y);
		var positionY:Number = m_position.y + (m_R.col1.y * m_localOBB.center.x + m_R.col2.y * m_localOBB.center.y);
		
		//aabb.minVertex = b2Math.SubtractVV(m_position, h);
		aabb.minVertex.x = positionX - hX;
		aabb.minVertex.y = positionY - hY;
		//aabb.maxVertex = b2Math.AddVV(m_position, h);
		aabb.maxVertex.x = positionX + hX;
		aabb.maxVertex.y = positionY + hY;
		
		var broadPhase:b2BroadPhase = m_body.m_world.m_broadPhase;
		if (broadPhase.InRange(aabb))
		{
			m_proxyId = broadPhase.CreateProxy(aabb, this);
		}
		else
		{
			m_proxyId = b2Pair.b2_nullProxy;
		}
		
		if (m_proxyId == b2Pair.b2_nullProxy)
		{
			m_body.Freeze();
		}
	}

	// Temp AABB for Synch function
	private var syncAABB:b2AABB = new b2AABB();
	private var syncMat:b2Mat22 = new b2Mat22();
	public override function Synchronize(position1:b2Vec2, R1:b2Mat22,
										position2:b2Vec2, R2:b2Mat22) : void{
		// The body transform is copied for convenience.
		m_R.SetM(R2);
		//m_position = m_body->m_position + b2Mul(m_body->m_R, m_localCentroid)
		m_position.x = m_body.m_position.x + (R2.col1.x * m_localCentroid.x + R2.col2.x * m_localCentroid.y);
		m_position.y = m_body.m_position.y + (R2.col1.y * m_localCentroid.x + R2.col2.y * m_localCentroid.y);
		
		if (m_proxyId == b2Pair.b2_nullProxy)
		{	
			return;
		}
		
		//b2AABB aabb1, aabb2;
		var hX:Number;
		var hY:Number;
		
		//b2Mat22 obbR = b2Mul(R1, m_localOBB.R);
			var v1:b2Vec2 = R1.col1;
			var v2:b2Vec2 = R1.col2;
			var v3:b2Vec2 = m_localOBB.R.col1;
			var v4:b2Vec2 = m_localOBB.R.col2;
			//syncMat.col1 = b2MulMV(R1, m_localOBB.R.col1);
			syncMat.col1.x = v1.x * v3.x + v2.x * v3.y;
			syncMat.col1.y = v1.y * v3.x + v2.y * v3.y;
			//syncMat.col2 = b2MulMV(R1, m_localOBB.R.col2);
			syncMat.col2.x = v1.x * v4.x + v2.x * v4.y;
			syncMat.col2.y = v1.y * v4.x + v2.y * v4.y;
		//b2Mat22 absR = b2Abs(obbR);
		syncMat.Abs();
		//b2Vec2 center = position1 + b2Mul(R1, m_localCentroid + m_localOBB.center);
		hX = m_localCentroid.x + m_localOBB.center.x;
		hY = m_localCentroid.y + m_localOBB.center.y;
		var centerX:Number = position1.x + (R1.col1.x * hX + R1.col2.x * hY);
		var centerY:Number = position1.y + (R1.col1.y * hX + R1.col2.y * hY);
		//b2Vec2 h = b2Mul(syncMat, m_localOBB.extents);
		hX = syncMat.col1.x * m_localOBB.extents.x + syncMat.col2.x * m_localOBB.extents.y;
		hY = syncMat.col1.y * m_localOBB.extents.x + syncMat.col2.y * m_localOBB.extents.y;
		//aabb1.minVertex = center - h;
		syncAABB.minVertex.x = centerX - hX;
		syncAABB.minVertex.y = centerY - hY;
		//aabb1.maxVertex = center + h;
		syncAABB.maxVertex.x = centerX + hX;
		syncAABB.maxVertex.y = centerY + hY;
		
		//b2Mat22 obbR = b2Mul(R2, m_localOBB.R);
			v1 = R2.col1;
			v2 = R2.col2;
			v3 = m_localOBB.R.col1;
			v4 = m_localOBB.R.col2;
			//syncMat.col1 = b2MulMV(R1, m_localOBB.R.col1);
			syncMat.col1.x = v1.x * v3.x + v2.x * v3.y;
			syncMat.col1.y = v1.y * v3.x + v2.y * v3.y;
			//syncMat.col2 = b2MulMV(R1, m_localOBB.R.col2);
			syncMat.col2.x = v1.x * v4.x + v2.x * v4.y;
			syncMat.col2.y = v1.y * v4.x + v2.y * v4.y;
		//b2Mat22 absR = b2Abs(obbR);
		syncMat.Abs();
		//b2Vec2 center = position2 + b2Mul(R2, m_localCentroid + m_localOBB.center);
		hX = m_localCentroid.x + m_localOBB.center.x;
		hY = m_localCentroid.y + m_localOBB.center.y;
		centerX = position2.x + (R2.col1.x * hX + R2.col2.x * hY);
		centerY = position2.y + (R2.col1.y * hX + R2.col2.y * hY);
		//b2Vec2 h = b2Mul(absR, m_localOBB.extents);
		hX = syncMat.col1.x * m_localOBB.extents.x + syncMat.col2.x * m_localOBB.extents.y;
		hY = syncMat.col1.y * m_localOBB.extents.x + syncMat.col2.y * m_localOBB.extents.y;
		//aabb2.minVertex = center - h;
		//aabb2.maxVertex = center + h;
		
		//aabb.minVertex = b2Min(aabb1.minVertex, aabb2.minVertex);
		syncAABB.minVertex.x = Math.min(syncAABB.minVertex.x, centerX - hX);
		syncAABB.minVertex.y = Math.min(syncAABB.minVertex.y, centerY - hY);
		//aabb.maxVertex = b2Max(aabb1.maxVertex, aabb2.maxVertex);
		syncAABB.maxVertex.x = Math.max(syncAABB.maxVertex.x, centerX + hX);
		syncAABB.maxVertex.y = Math.max(syncAABB.maxVertex.y, centerY + hY);
		
		var broadPhase:b2BroadPhase = m_body.m_world.m_broadPhase;
		if (broadPhase.InRange(syncAABB))
		{
			broadPhase.MoveProxy(m_proxyId, syncAABB);
		}
		else
		{
			m_body.Freeze();
		}
	}
	
	public override function QuickSync(position:b2Vec2, R:b2Mat22) : void{
		//m_R = R;
		m_R.SetM(R);
		//m_position = position + b2Mul(R, m_localCentroid);
		m_position.x = position.x + (R.col1.x * m_localCentroid.x + R.col2.x * m_localCentroid.y);
		m_position.y = position.y + (R.col1.y * m_localCentroid.x + R.col2.y * m_localCentroid.y);
	}
	
	public override function ResetProxy(broadPhase:b2BroadPhase) : void{
		
		if (m_proxyId == b2Pair.b2_nullProxy)
		{	
			return;
		}
		
		var proxy:b2Proxy = broadPhase.GetProxy(m_proxyId);
		
		broadPhase.DestroyProxy(m_proxyId);
		proxy = null;
		
		var R:b2Mat22 = b2Math.b2MulMM(m_R, m_localOBB.R);
		var absR:b2Mat22 = b2Math.b2AbsM(R);
		var h:b2Vec2 = b2Math.b2MulMV(absR, m_localOBB.extents);
		//var position:b2Vec2 = m_position + b2Mul(m_R, m_localOBB.center);
		var position:b2Vec2 = b2Math.b2MulMV(m_R, m_localOBB.center);
		position.Add(m_position);
		
		var aabb:b2AABB = new b2AABB();
		//aabb.minVertex = position - h;
		aabb.minVertex.SetV(position);
		aabb.minVertex.Subtract(h);
		//aabb.maxVertex = position + h;
		aabb.maxVertex.SetV(position);
		aabb.maxVertex.Add(h);
		
		if (broadPhase.InRange(aabb))
		{
			m_proxyId = broadPhase.CreateProxy(aabb, this);
		}
		else
		{
			m_proxyId = b2Pair.b2_nullProxy;
		}
		
		if (m_proxyId == b2Pair.b2_nullProxy)
		{
			m_body.Freeze();
		}
	}
	
	
	public override function Support(dX:Number, dY:Number, out:b2Vec2):void
	{
		//b2Vec2 dLocal = b2MulT(m_R, d);
		var dLocalX:Number = (dX*m_R.col1.x + dY*m_R.col1.y);
		var dLocalY:Number = (dX*m_R.col2.x + dY*m_R.col2.y);
		
		var bestIndex:int = 0;
		//float32 bestValue = b2Dot(m_vertices[0], dLocal);
		var bestValue:Number = (m_coreVertices[0].x * dLocalX + m_coreVertices[0].y * dLocalY);
		for (var i:int = 1; i < m_vertexCount; ++i)
		{
			//float32 value = b2Dot(m_vertices[i], dLocal);
			var value:Number = (m_coreVertices[i].x * dLocalX + m_coreVertices[i].y * dLocalY);
			if (value > bestValue)
			{
				bestIndex = i;
				bestValue = value;
			}
		}
		
		//return m_position + b2Mul(m_R, m_vertices[bestIndex]);
		out.Set(	m_position.x + (m_R.col1.x * m_coreVertices[bestIndex].x + m_R.col2.x * m_coreVertices[bestIndex].y),
					m_position.y + (m_R.col1.y * m_coreVertices[bestIndex].x + m_R.col2.y * m_coreVertices[bestIndex].y));
		
	}
	
	
	// Local position of the shape centroid in parent body frame.
	public var m_localCentroid:b2Vec2 = new b2Vec2();

	// Local position oriented bounding box. The OBB center is relative to
	// shape centroid.
	public var m_localOBB:b2OBB = new b2OBB();
	public var m_vertices:Array;
	public var m_coreVertices:Array;
	public var m_vertexCount:int;
	public var m_normals:Array;
};

}