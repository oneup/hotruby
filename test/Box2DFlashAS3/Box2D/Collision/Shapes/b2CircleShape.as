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



public class b2CircleShape extends b2Shape
{
	public override function TestPoint(p:b2Vec2):Boolean{
		//var d:b2Vec2 = b2Math.SubtractVV(p, m_position);
		var d:b2Vec2 = new b2Vec2();
		d.SetV(p);
		d.Subtract(m_position);
		return b2Math.b2Dot(d, d) <= m_radius * m_radius;
	}

	//--------------- Internals Below -------------------

	public function b2CircleShape(def:b2ShapeDef, body:b2Body, localCenter:b2Vec2){
		super(def, body);
		
		//b2Settings.b2Assert(def.type == b2Shape.e_circleShape);
		var circle:b2CircleDef = def as b2CircleDef;
		
		//m_localPosition = def.localPosition - localCenter;
		m_localPosition.Set(def.localPosition.x - localCenter.x, def.localPosition.y - localCenter.y);
		m_type = b2Shape.e_circleShape;
		m_radius = circle.radius;
		
		m_R.SetM(m_body.m_R);
		//b2Vec2 r = b2Mul(m_body->m_R, m_localPosition);
		var rX:Number = m_R.col1.x * m_localPosition.x + m_R.col2.x * m_localPosition.y;
		var rY:Number = m_R.col1.y * m_localPosition.x + m_R.col2.y * m_localPosition.y;
		//m_position = m_body->m_position + r;
		m_position.x = m_body.m_position.x + rX;
		m_position.y = m_body.m_position.y + rY;
		//m_maxRadius = r.Length() + m_radius;
		m_maxRadius = Math.sqrt(rX*rX+rY*rY) + m_radius;
		
		var aabb:b2AABB = new b2AABB();
		aabb.minVertex.Set(m_position.x - m_radius, m_position.y - m_radius);
		aabb.maxVertex.Set(m_position.x + m_radius, m_position.y + m_radius);
		
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

	public override function Synchronize(position1:b2Vec2, R1:b2Mat22,
										position2:b2Vec2, R2:b2Mat22) : void{
		m_R.SetM(R2);
		//m_position = position2 + b2Mul(R2, m_localPosition);
		m_position.x = (R2.col1.x * m_localPosition.x + R2.col2.x * m_localPosition.y) + position2.x;
		m_position.y = (R2.col1.y * m_localPosition.x + R2.col2.y * m_localPosition.y) + position2.y;
		
		if (m_proxyId == b2Pair.b2_nullProxy)
		{	
			return;
		}
		
		// Compute an AABB that covers the swept shape (may miss some rotation effect).
		//b2Vec2 p1 = position1 + b2Mul(R1, m_localPosition);
		var p1X:Number = position1.x + (R1.col1.x * m_localPosition.x + R1.col2.x * m_localPosition.y);
		var p1Y:Number = position1.y + (R1.col1.y * m_localPosition.x + R1.col2.y * m_localPosition.y);
		//b2Vec2 lower = b2Min(p1, m_position);
		var lowerX:Number = Math.min(p1X, m_position.x);
		var lowerY:Number = Math.min(p1Y, m_position.y);
		//b2Vec2 upper = b2Max(p1, m_position);
		var upperX:Number = Math.max(p1X, m_position.x);
		var upperY:Number = Math.max(p1Y, m_position.y);
		
		var aabb:b2AABB = new b2AABB();
		aabb.minVertex.Set(lowerX - m_radius, lowerY - m_radius);
		aabb.maxVertex.Set(upperX + m_radius, upperY + m_radius);
		
		var broadPhase:b2BroadPhase = m_body.m_world.m_broadPhase;
		if (broadPhase.InRange(aabb))
		{
			broadPhase.MoveProxy(m_proxyId, aabb);
		}
		else
		{
			m_body.Freeze();
		}
	}
	
	public override function QuickSync(position:b2Vec2, R:b2Mat22) : void{
		m_R.SetM(R);
		//m_position = position + b2Mul(R, m_localPosition);
		m_position.x = (R.col1.x * m_localPosition.x + R.col2.x * m_localPosition.y) + position.x;
		m_position.y = (R.col1.y * m_localPosition.x + R.col2.y * m_localPosition.y) + position.y;
	}
	
	
	public override function ResetProxy(broadPhase:b2BroadPhase) : void
	{
		if (m_proxyId == b2Pair.b2_nullProxy)
		{	
			return;
		}
		
		var proxy:b2Proxy = broadPhase.GetProxy(m_proxyId);
		
		broadPhase.DestroyProxy(m_proxyId);
		proxy = null;
		
		var aabb:b2AABB = new b2AABB();
		aabb.minVertex.Set(m_position.x - m_radius, m_position.y - m_radius);
		aabb.maxVertex.Set(m_position.x + m_radius, m_position.y + m_radius);
		
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
	
	
	public override function Support(dX:Number, dY:Number, out:b2Vec2): void
	{
		//b2Vec2 u = d;
		//u.Normalize();
		var len:Number = Math.sqrt(dX*dX + dY*dY);
		dX /= len;
		dY /= len;
		//return m_position + m_radius * u;
		out.Set(	m_position.x + m_radius*dX, 
					m_position.y + m_radius*dY);
	}
	
	
	// Local position in parent body
	public var m_localPosition:b2Vec2 = new b2Vec2();
	public var m_radius:Number;
};

}