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

package Box2D.Dynamics.Joints{
	
import Box2D.Common.Math.*
import Box2D.Common.*
import Box2D.Dynamics.*;

// C = norm(p2 - p1) - L
// u = (p2 - p1) / norm(p2 - p1)
// Cdot = dot(u, v2 + cross(w2, r2) - v1 - cross(w1, r1))
// J = [-u -cross(r1, u) u cross(r2, u)]
// K = J * invM * JT
//   = invMass1 + invI1 * cross(r1, u)^2 + invMass2 + invI2 * cross(r2, u)^2

public class b2DistanceJoint extends b2Joint
{
	//--------------- Internals Below -------------------

	public function b2DistanceJoint(def:b2DistanceJointDef){
		super(def);
		
		var tMat:b2Mat22;
		var tX:Number;
		var tY:Number;
		//m_localAnchor1 = b2MulT(m_body1->m_R, def->anchorPoint1 - m_body1->m_position);
		tMat = m_body1.m_R;
		tX = def.anchorPoint1.x - m_body1.m_position.x;
		tY = def.anchorPoint1.y - m_body1.m_position.y;
		m_localAnchor1.x = tX*tMat.col1.x + tY*tMat.col1.y;
		m_localAnchor1.y = tX*tMat.col2.x + tY*tMat.col2.y;
		//m_localAnchor2 = b2MulT(m_body2->m_R, def->anchorPoint2 - m_body2->m_position);
		tMat = m_body2.m_R;
		tX = def.anchorPoint2.x - m_body2.m_position.x;
		tY = def.anchorPoint2.y - m_body2.m_position.y;
		m_localAnchor2.x = tX*tMat.col1.x + tY*tMat.col1.y;
		m_localAnchor2.y = tX*tMat.col2.x + tY*tMat.col2.y;
		
		//b2Vec2 d = def->anchorPoint2 - def->anchorPoint1;
		tX = def.anchorPoint2.x - def.anchorPoint1.x;
		tY = def.anchorPoint2.y - def.anchorPoint1.y;
		//m_length = d.Length();
		m_length = Math.sqrt(tX*tX + tY*tY);
		m_impulse = 0.0;
	}

	public override function PrepareVelocitySolver() : void{
		
		var tMat:b2Mat22;
		
		// Compute the effective mass matrix.
		//b2Vec2 r1 = b2Mul(m_body1->m_R, m_localAnchor1);
		tMat = m_body1.m_R;
		var r1X:Number = tMat.col1.x * m_localAnchor1.x + tMat.col2.x * m_localAnchor1.y;
		var r1Y:Number = tMat.col1.y * m_localAnchor1.x + tMat.col2.y * m_localAnchor1.y;
		//b2Vec2 r2 = b2Mul(m_body2->m_R, m_localAnchor2);
		tMat = m_body2.m_R;
		var r2X:Number = tMat.col1.x * m_localAnchor2.x + tMat.col2.x * m_localAnchor2.y;
		var r2Y:Number = tMat.col1.y * m_localAnchor2.x + tMat.col2.y * m_localAnchor2.y;
		//m_u = m_body2->m_position + r2 - m_body1->m_position - r1;
		m_u.x = m_body2.m_position.x + r2X - m_body1.m_position.x - r1X;
		m_u.y = m_body2.m_position.y + r2Y - m_body1.m_position.y - r1Y;
		
		// Handle singularity.
		//float32 length = m_u.Length();
		var length:Number = Math.sqrt(m_u.x*m_u.x + m_u.y*m_u.y);
		if (length > b2Settings.b2_linearSlop)
		{
			//m_u *= 1.0 / length;
			m_u.Multiply( 1.0 / length );
		}
		else
		{
			m_u.SetZero();
		}
		
		//float32 cr1u = b2Cross(r1, m_u);
		var cr1u:Number = (r1X * m_u.y - r1Y * m_u.x);
		//float32 cr2u = b2Cross(r2, m_u);
		var cr2u:Number = (r2X * m_u.y - r2Y * m_u.x);
		//m_mass = m_body1->m_invMass + m_body1->m_invI * cr1u * cr1u + m_body2->m_invMass + m_body2->m_invI * cr2u * cr2u;
		m_mass = m_body1.m_invMass + m_body1.m_invI * cr1u * cr1u + m_body2.m_invMass + m_body2.m_invI * cr2u * cr2u;
		//b2Settings.b2Assert(m_mass > Number.MIN_VALUE);
		m_mass = 1.0 / m_mass;
		
		if (b2World.s_enableWarmStarting)
		{
			//b2Vec2 P = m_impulse * m_u;
			var PX:Number = m_impulse * m_u.x;
			var PY:Number = m_impulse * m_u.y;
			//m_body1.m_linearVelocity -= m_body1.m_invMass * P;
			m_body1.m_linearVelocity.x -= m_body1.m_invMass * PX;
			m_body1.m_linearVelocity.y -= m_body1.m_invMass * PY;
			//m_body1.m_angularVelocity -= m_body1.m_invI * b2Cross(r1, P);
			m_body1.m_angularVelocity -= m_body1.m_invI * (r1X * PY - r1Y * PX);
			//m_body2.m_linearVelocity += m_body2.m_invMass * P;
			m_body2.m_linearVelocity.x += m_body2.m_invMass * PX;
			m_body2.m_linearVelocity.y += m_body2.m_invMass * PY;
			//m_body2.m_angularVelocity += m_body2.m_invI * b2Cross(r2, P);
			m_body2.m_angularVelocity += m_body2.m_invI * (r2X * PY - r2Y * PX);
		}
		else
		{
			m_impulse = 0.0;
		}
		
	}
	
	
	
	public override function SolveVelocityConstraints(step:b2TimeStep): void{
		
		var tMat:b2Mat22;
		
		//b2Vec2 r1 = b2Mul(m_body1->m_R, m_localAnchor1);
		tMat = m_body1.m_R;
		var r1X:Number = tMat.col1.x * m_localAnchor1.x + tMat.col2.x * m_localAnchor1.y;
		var r1Y:Number = tMat.col1.y * m_localAnchor1.x + tMat.col2.y * m_localAnchor1.y;
		//b2Vec2 r2 = b2Mul(m_body2->m_R, m_localAnchor2);
		tMat = m_body2.m_R;
		var r2X:Number = tMat.col1.x * m_localAnchor2.x + tMat.col2.x * m_localAnchor2.y;
		var r2Y:Number = tMat.col1.y * m_localAnchor2.x + tMat.col2.y * m_localAnchor2.y;
		
		// Cdot = dot(u, v + cross(w, r))
		//b2Vec2 v1 = m_body1->m_linearVelocity + b2Cross(m_body1->m_angularVelocity, r1);
		var v1X:Number = m_body1.m_linearVelocity.x + (-m_body1.m_angularVelocity * r1Y);
		var v1Y:Number = m_body1.m_linearVelocity.y + (m_body1.m_angularVelocity * r1X);
		//b2Vec2 v2 = m_body2->m_linearVelocity + b2Cross(m_body2->m_angularVelocity, r2);
		var v2X:Number = m_body2.m_linearVelocity.x + (-m_body2.m_angularVelocity * r2Y);
		var v2Y:Number = m_body2.m_linearVelocity.y + (m_body2.m_angularVelocity * r2X);
		//float32 Cdot = b2Dot(m_u, v2 - v1);
		var Cdot:Number = (m_u.x * (v2X - v1X) + m_u.y * (v2Y - v1Y));
		//float32 impulse = -m_mass * Cdot;
		var impulse:Number = -m_mass * Cdot;
		m_impulse += impulse;
		
		//b2Vec2 P = impulse * m_u;
		var PX:Number = impulse * m_u.x;
		var PY:Number = impulse * m_u.y;
		//m_body1->m_linearVelocity -= m_body1->m_invMass * P;
		m_body1.m_linearVelocity.x -= m_body1.m_invMass * PX;
		m_body1.m_linearVelocity.y -= m_body1.m_invMass * PY;
		//m_body1->m_angularVelocity -= m_body1->m_invI * b2Cross(r1, P);
		m_body1.m_angularVelocity -= m_body1.m_invI * (r1X * PY - r1Y * PX);
		//m_body2->m_linearVelocity += m_body2->m_invMass * P;
		m_body2.m_linearVelocity.x += m_body2.m_invMass * PX;
		m_body2.m_linearVelocity.y += m_body2.m_invMass * PY;
		//m_body2->m_angularVelocity += m_body2->m_invI * b2Cross(r2, P);
		m_body2.m_angularVelocity += m_body2.m_invI * (r2X * PY - r2Y * PX);
	}
	
	public override function SolvePositionConstraints():Boolean{
		
		var tMat:b2Mat22;
		
		//b2Vec2 r1 = b2Mul(m_body1->m_R, m_localAnchor1);
		tMat = m_body1.m_R;
		var r1X:Number = tMat.col1.x * m_localAnchor1.x + tMat.col2.x * m_localAnchor1.y;
		var r1Y:Number = tMat.col1.y * m_localAnchor1.x + tMat.col2.y * m_localAnchor1.y;
		//b2Vec2 r2 = b2Mul(m_body2->m_R, m_localAnchor2);
		tMat = m_body2.m_R;
		var r2X:Number = tMat.col1.x * m_localAnchor2.x + tMat.col2.x * m_localAnchor2.y;
		var r2Y:Number = tMat.col1.y * m_localAnchor2.x + tMat.col2.y * m_localAnchor2.y;
		//b2Vec2 d = m_body2->m_position + r2 - m_body1->m_position - r1;
		var dX:Number = m_body2.m_position.x + r2X - m_body1.m_position.x - r1X;
		var dY:Number = m_body2.m_position.y + r2Y - m_body1.m_position.y - r1Y;
		
		//float32 length = d.Normalize();
		var length:Number = Math.sqrt(dX*dX + dY*dY);
		dX /= length;
		dY /= length;
		//float32 C = length - m_length;
		var C:Number = length - m_length;
		C = b2Math.b2Clamp(C, -b2Settings.b2_maxLinearCorrection, b2Settings.b2_maxLinearCorrection);
		
		var impulse:Number = -m_mass * C;
		//m_u = d;
		m_u.Set(dX, dY);
		//b2Vec2 P = impulse * m_u;
		var PX:Number = impulse * m_u.x;
		var PY:Number = impulse * m_u.y;
		
		//m_body1->m_position -= m_body1->m_invMass * P;
		m_body1.m_position.x -= m_body1.m_invMass * PX;
		m_body1.m_position.y -= m_body1.m_invMass * PY;
		//m_body1->m_rotation -= m_body1->m_invI * b2Cross(r1, P);
		m_body1.m_rotation -= m_body1.m_invI * (r1X * PY - r1Y * PX);
		//m_body2->m_position += m_body2->m_invMass * P;
		m_body2.m_position.x += m_body2.m_invMass * PX;
		m_body2.m_position.y += m_body2.m_invMass * PY;
		//m_body2->m_rotation -= m_body2->m_invI * b2Cross(r2, P);
		m_body2.m_rotation += m_body2.m_invI * (r2X * PY - r2Y * PX);
		
		m_body1.m_R.Set(m_body1.m_rotation);
		m_body2.m_R.Set(m_body2.m_rotation);
		
		return b2Math.b2Abs(C) < b2Settings.b2_linearSlop;
		
	}
	
	public override function GetAnchor1():b2Vec2{
		return b2Math.AddVV(m_body1.m_position , b2Math.b2MulMV(m_body1.m_R, m_localAnchor1));
	}
	public override function GetAnchor2():b2Vec2{
		return b2Math.AddVV(m_body2.m_position , b2Math.b2MulMV(m_body2.m_R, m_localAnchor2));
	}
	
	public override function GetReactionForce(invTimeStep:Number):b2Vec2
	{
		//var F:b2Vec2 = (m_impulse * invTimeStep) * m_u;
		var F:b2Vec2 = new b2Vec2();
		F.SetV(m_u);
		F.Multiply(m_impulse * invTimeStep);
		return F;
	}

	public override function GetReactionTorque(invTimeStep:Number):Number
	{
		//NOT_USED(invTimeStep);
		return 0.0;
	}

	public var m_localAnchor1:b2Vec2 = new b2Vec2();
	public var m_localAnchor2:b2Vec2 = new b2Vec2();
	public var m_u:b2Vec2 = new b2Vec2();
	public var m_impulse:Number;
	public var m_mass:Number;	// effective mass for the constraint.
	public var m_length:Number;
};

}
