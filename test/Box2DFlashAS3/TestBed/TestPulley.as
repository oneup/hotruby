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
	
	
	
	public class TestPulley extends Test{
		
		public function TestPulley(){
			
			// Set Text field
			Main.m_aboutText.text = "Pulley";
			
			var ground:b2Body = m_world.m_groundBody;
			
			var sd:b2BoxDef = new b2BoxDef();
			sd.extents.Set(50 / m_physScale, 20 / m_physScale);
			sd.density = 5.0;
			
			var bd:b2BodyDef = new b2BodyDef();
			bd.AddShape(sd);
			
			bd.position.Set(180 / m_physScale, 200 / m_physScale);
			var body1:b2Body = m_world.CreateBody(bd);
			
			bd.position.Set(460 / m_physScale, 200 / m_physScale);
			var body2:b2Body = m_world.CreateBody(bd);
			
			var pulleyDef:b2PulleyJointDef = new b2PulleyJointDef();
			pulleyDef.body1 = body1;
			
			pulleyDef.body2 = body2;
			pulleyDef.anchorPoint1.Set(180 / m_physScale, 180 / m_physScale);
			pulleyDef.anchorPoint2.Set(460 / m_physScale, 180 / m_physScale);
			pulleyDef.groundPoint1.Set(180 / m_physScale, 50 / m_physScale);
			pulleyDef.groundPoint2.Set(460 / m_physScale, 50 / m_physScale);
			pulleyDef.ratio = 2.0;
			
			pulleyDef.maxLength1 = 200 / m_physScale;
			pulleyDef.maxLength2 = 150 / m_physScale;
			
			//m_joint1 = m_world.CreateJoint(pulleyDef) as b2PulleyJoint;
			m_world.CreateJoint(pulleyDef) as b2PulleyJoint;
			
			var prismDef:b2PrismaticJointDef = new b2PrismaticJointDef();
			prismDef.body1 = ground;
			prismDef.body2 = body2;
			prismDef.axis.Set(0.0, -1.0);
			prismDef.anchorPoint = body2.GetCenterPosition();
			//m_joint2 = m_world.CreateJoint(prismDef) as b2PrismaticJoint;
			m_world.CreateJoint(prismDef) as b2PrismaticJoint;
			
			
			// Add some falling boxes to make it interesting :P
			var cd:b2CircleDef = new b2CircleDef();
			bd = new b2BodyDef();
			bd.AddShape(cd);
			cd.radius = 20 / m_physScale;
			cd.density = 20.0;
			cd.restitution = 0.3;
			bd.position.Set(460 / m_physScale, 30 / m_physScale);
			bd.rotation = Math.random();
			m_world.CreateBody(bd);
			cd.restitution = 0.1;
			bd.position.Set(430 / m_physScale, 20 / m_physScale);
			bd.rotation = Math.random();
			m_world.CreateBody(bd);
			cd.restitution = 0.5;
			bd.position.Set(490 / m_physScale, 40 / m_physScale);
			bd.rotation = Math.random();
			m_world.CreateBody(bd);
			
			
			
		}
		
		
		//======================
		// Member Data 
		//======================
	}
	
}