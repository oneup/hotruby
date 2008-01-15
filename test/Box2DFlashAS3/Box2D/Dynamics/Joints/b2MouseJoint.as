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
import Box2D.Dynamics.*
	

// p = attached point, m = mouse point
// C = p - m
// Cdot = v
//      = v + cross(w, r)
// J = [I r_skew]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

public class b2MouseJoint extends b2Joint
{
	public override function GetAnchor1():b2Vec2{
		return m_target;
	}
	public override function GetAnchor2():b2Vec2{
		var tVec:b2Vec2 = b2Math.b2MulMV(m_body2.m_R, m_localAnchor);
		tVec.Add(m_body2.m_position);
		return tVec;
	}
	
	public override function GetReactionForce(invTimeStep:Number):b2Vec2
	{
		//b2Vec2 F = invTimeStep * m_impulse;
		var F:b2Vec2 = new b2Vec2();
		F.SetV(m_impulse);
		F.Multiply(invTimeStep);
		return F;
	}

	public override function GetReactionTorque(invTimeStep:Number):Number
	{
		//NOT_USED(invTimeStep);
		return 0.0;
	}
	
	public function SetTarget(target:b2Vec2) : void{
		m_body2.WakeUp();
		m_target = target;
	}

	//--------------- Internals Below -------------------

	public function b2MouseJoint(def:b2MouseJointDef){
		super(def);
		
		m_target.SetV(def.target);
		//m_localAnchor = b2Math.b2MulTMV(m_body2.m_R, b2Math.SubtractVV( m_target, m_body2.m_position ) );
		var tX:Number = m_target.x - m_body2.m_position.x;
		var tY:Number = m_target.y - m_body2.m_position.y;
		m_localAnchor.x = (tX * m_body2.m_R.col1.x + tY * m_body2.m_R.col1.y);
		m_localAnchor.y = (tX * m_body2.m_R.col2.x + tY * m_body2.m_R.col2.y);
		
		m_maxForce = def.maxForce;
		m_impulse.SetZero();
		
		var mass:Number = m_body2.m_mass;
		
		// Frequency
		var omega:Number = 2.0 * b2Settings.b2_pi * def.frequencyHz;
		
		// Damping coefficient
		var d:Number = 2.0 * mass * def.dampingRatio * omega;
		
		// Spring stiffness
		var k:Number = mass * omega * omega;
		
		// magic formulas
		m_gamma = 1.0 / (d + def.timeStep * k);
		m_beta = def.timeStep * k / (d + def.timeStep * k);
	}

	// Presolve vars
	private var K:b2Mat22 = new b2Mat22();
	private var K1:b2Mat22 = new b2Mat22();
	private var K2:b2Mat22 = new b2Mat22();
	public override function PrepareVelocitySolver(): void{
		var b:b2Body = m_body2;
		
		var tMat:b2Mat22;
		
		// Compute the effective mass matrix.
		//b2Vec2 r = b2Mul(b.m_R, m_localAnchor);
		tMat = b.m_R;
		var rX:Number = tMat.col1.x * m_localAnchor.x + tMat.col2.x * m_localAnchor.y;
		var rY:Number = tMat.col1.y * m_localAnchor.x + tMat.col2.y * m_localAnchor.y;
		
		// K    = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
		//      = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
		//        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
		var invMass:Number = b.m_invMass;
		var invI:Number = b.m_invI;
		
		//b2Mat22 K1;
		K1.col1.x = invMass;	K1.col2.x = 0.0;
		K1.col1.y = 0.0;		K1.col2.y = invMass;
		
		//b2Mat22 K2;
		K2.col1.x =  invI * rY * rY;	K2.col2.x = -invI * rX * rY;
		K2.col1.y = -invI * rX * rY;	K2.col2.y =  invI * rX * rX;
		
		//b2Mat22 K = K1 + K2;
		K.SetM(K1);
		K.AddM(K2);
		K.col1.x += m_gamma;
		K.col2.y += m_gamma;
		
		//m_ptpMass = K.Invert();
		K.Invert(m_ptpMass);
		
		//m_C = b.m_position + r - m_target;
		m_C.x = b.m_position.x + rX - m_target.x;
		m_C.y = b.m_position.y + rY - m_target.y;
		
		// Cheat with some damping
		b.m_angularVelocity *= 0.98;
		
		// Warm starting.
		//b2Vec2 P = m_impulse;
		var PX:Number = m_impulse.x;
		var PY:Number = m_impulse.y;
		//b.m_linearVelocity += invMass * P;
		b.m_linearVelocity.x += invMass * PX;
		b.m_linearVelocity.y += invMass * PY;
		//b.m_angularVelocity += invI * b2Cross(r, P);
		b.m_angularVelocity += invI * (rX * PY - rY * PX);
	}
	
	
	public override function SolveVelocityConstraints(step:b2TimeStep) : void{
		var body:b2Body = m_body2;
		
		var tMat:b2Mat22;
		
		// Compute the effective mass matrix.
		//b2Vec2 r = b2Mul(body.m_R, m_localAnchor);
		tMat = body.m_R;
		var rX:Number = tMat.col1.x * m_localAnchor.x + tMat.col2.x * m_localAnchor.y;
		var rY:Number = tMat.col1.y * m_localAnchor.x + tMat.col2.y * m_localAnchor.y;
		
		// Cdot = v + cross(w, r)
		//b2Vec2 Cdot = body->m_linearVelocity + b2Cross(body->m_angularVelocity, r);
		var CdotX:Number = body.m_linearVelocity.x + (-body.m_angularVelocity * rY);
		var CdotY:Number = body.m_linearVelocity.y + (body.m_angularVelocity * rX);
		//b2Vec2 impulse = -b2Mul(m_ptpMass, Cdot + (m_beta * step->inv_dt) * m_C + m_gamma * m_impulse);
		tMat = m_ptpMass;
		var tX:Number = CdotX + (m_beta * step.inv_dt) * m_C.x + m_gamma * m_impulse.x;
		var tY:Number = CdotY + (m_beta * step.inv_dt) * m_C.y + m_gamma * m_impulse.y;
		var impulseX:Number = -(tMat.col1.x * tX + tMat.col2.x * tY);
		var impulseY:Number = -(tMat.col1.y * tX + tMat.col2.y * tY);
		
		var oldImpulseX:Number = m_impulse.x;
		var oldImpulseY:Number = m_impulse.y;
		//m_impulse += impulse;
		m_impulse.x += impulseX;
		m_impulse.y += impulseY;
		var length:Number = m_impulse.Length();
		if (length > step.dt * m_maxForce)
		{
			//m_impulse *= step.dt * m_maxForce / length;
			m_impulse.Multiply(step.dt * m_maxForce / length);
		}
		//impulse = m_impulse - oldImpulse;
		impulseX = m_impulse.x - oldImpulseX;
		impulseY = m_impulse.y - oldImpulseY;
		
		//body.m_linearVelocity += body->m_invMass * impulse;
		body.m_linearVelocity.x += body.m_invMass * impulseX;
		body.m_linearVelocity.y += body.m_invMass * impulseY;
		//body.m_angularVelocity += body->m_invI * b2Cross(r, impulse);
		body.m_angularVelocity += body.m_invI * (rX * impulseY - rY * impulseX);
	}
	public override function SolvePositionConstraints():Boolean { 
		return true; 
	}

	public var m_localAnchor:b2Vec2 = new b2Vec2();
	public var m_target:b2Vec2 = new b2Vec2();
	public var m_impulse:b2Vec2 = new b2Vec2();

	public var m_ptpMass:b2Mat22 = new b2Mat22();	// effective mass for point-to-point constraint.
	public var m_C:b2Vec2 = new b2Vec2();			// position error
	public var m_maxForce:Number;
	public var m_beta:Number;						// bias factor
	public var m_gamma:Number;						// softness
};

}
