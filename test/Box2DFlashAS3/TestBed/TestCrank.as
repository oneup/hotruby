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


package TestBed{
	
	
	
	import Box2D.Dynamics.*;
	import Box2D.Collision.*;
	import Box2D.Collision.Shapes.*;
	import Box2D.Dynamics.Joints.*;
	import Box2D.Dynamics.Contacts.*;
	import Box2D.Common.*;
	import Box2D.Common.Math.*;
	
	
	
	public class TestCrank extends Test{
		
		public function TestCrank(){
			
			// Set Text field
			Main.m_aboutText.text = "Slider Crank";
			
			var ground:b2Body = m_world.m_groundBody;
			
			// Define crank.
			var sd:b2BoxDef = new b2BoxDef();
			sd.extents.Set(5 / m_physScale, 25 / m_physScale);
			sd.density = 1.0;
			
			var bd:b2BodyDef = new b2BodyDef();
			bd.AddShape(sd);
			
			var rjd:b2RevoluteJointDef = new b2RevoluteJointDef();
			
			var prevBody:b2Body = ground;
			
			bd.position.Set(640/2 / m_physScale, 250 / m_physScale);
			var body:b2Body = m_world.CreateBody(bd);
			
			rjd.anchorPoint.Set(640/2 / m_physScale, 275 / m_physScale);
			rjd.body1 = prevBody;
			rjd.body2 = body;
			rjd.motorSpeed = -1.0 * Math.PI;
			rjd.motorTorque = 500000000.0;
			rjd.enableMotor = true;
			m_joint1 = m_world.CreateJoint(rjd) as b2RevoluteJoint;
			
			prevBody = body;
			
			// Define follower.
			sd.extents.Set(5 / m_physScale, 45 / m_physScale);
			bd.position.Set(640/2 / m_physScale, 180 / m_physScale);
			body = m_world.CreateBody(bd);
			
			rjd.anchorPoint.Set(640/2 / m_physScale, 225 / m_physScale);
			rjd.body1 = prevBody;
			rjd.body2 = body;
			rjd.enableMotor = false;
			m_world.CreateJoint(rjd);
			
			prevBody = body;
			
			// Define piston
			sd.extents.Set(20 / m_physScale, 20 / m_physScale);
			bd.position.Set(640/2 / m_physScale, 135 / m_physScale);
			body = m_world.CreateBody(bd);
			
			rjd.anchorPoint.Set(640/2 / m_physScale, 135 / m_physScale);
			rjd.body1 = prevBody;
			rjd.body2 = body;
			m_world.CreateJoint(rjd);
			
			var pjd:b2PrismaticJointDef = new b2PrismaticJointDef();
			pjd.anchorPoint.Set(640/2 / m_physScale, 135 / m_physScale);
			pjd.body1 = ground;
			pjd.body2 = body;
			pjd.axis.Set(0.0, 1.0);
			pjd.motorSpeed = 0.0;		// joint friction
			pjd.motorForce = 100000.0;
			pjd.enableMotor = true;
			
			m_joint2 = m_world.CreateJoint(pjd) as b2PrismaticJoint;
			
			// Create a payload
			sd.density = 2.0;
			bd.position.Set(640/2 / m_physScale, 50 / m_physScale);
			m_world.CreateBody(bd);
			
		}
		
		
		//======================
		// Member Data 
		//======================
		private var m_joint1:b2RevoluteJoint;
		private var m_joint2:b2PrismaticJoint;
		
	}
	
}