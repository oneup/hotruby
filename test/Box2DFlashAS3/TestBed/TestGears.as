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
	
	
	
	public class TestGears extends Test{
		
		public function TestGears(){
			
			// Set Text field
			Main.m_aboutText.text = "Gears";
			
			var ground:b2Body = m_world.m_groundBody;
			
			var circle1:b2CircleDef = new b2CircleDef();
			circle1.radius = 25 / m_physScale;
			circle1.density = 5.0;
			
			var circle2:b2CircleDef = new b2CircleDef();
			circle2.radius = 50 / m_physScale;
			circle2.density = 5.0;
			
			var box:b2BoxDef = new b2BoxDef();
			box.extents.Set(10 / m_physScale, 100 / m_physScale);
			box.density = 5.0;
			
			var bd1:b2BodyDef = new b2BodyDef();
			bd1.AddShape(circle1);
			bd1.position.Set(200 / m_physScale, 360/2 / m_physScale);
			var body1:b2Body = m_world.CreateBody(bd1);
			
			var jd1:b2RevoluteJointDef = new b2RevoluteJointDef();
			jd1.anchorPoint.SetV(bd1.position);
			jd1.body1 = ground;
			jd1.body2 = body1;
			m_joint1 = m_world.CreateJoint(jd1) as b2RevoluteJoint;
			
			var bd2:b2BodyDef = new b2BodyDef();
			bd2.AddShape(circle2);
			bd2.position.Set(275 / m_physScale, 360/2 / m_physScale);
			var body2:b2Body = m_world.CreateBody(bd2);
			
			var jd2:b2RevoluteJointDef = new b2RevoluteJointDef();
			jd2.body1 = ground;
			jd2.body2 = body2;
			jd2.anchorPoint.SetV(bd2.position);
			m_joint2 = m_world.CreateJoint(jd2) as b2RevoluteJoint;
			
			var bd3:b2BodyDef = new b2BodyDef();
			bd3.AddShape(box);
			bd3.position.Set(335 / m_physScale, 360/2 / m_physScale);
			var body3:b2Body = m_world.CreateBody(bd3);
			
			var jd3:b2PrismaticJointDef = new b2PrismaticJointDef();
			jd3.body1 = ground;
			jd3.body2 = body3;
			jd3.anchorPoint.SetV(bd3.position);
			jd3.axis.Set(0.0, 1.0);
			jd3.lowerTranslation = -25.0 / m_physScale;
			jd3.upperTranslation = 100.0 / m_physScale;
			jd3.enableLimit = true;
			
			m_joint3 = m_world.CreateJoint(jd3) as b2PrismaticJoint;
			
			var jd4:b2GearJointDef = new b2GearJointDef();
			jd4.body1 = body1;
			jd4.body2 = body2;
			jd4.joint1 = m_joint1;
			jd4.joint2 = m_joint2;
			jd4.ratio = circle2.radius / circle1.radius;
			m_joint4 = m_world.CreateJoint(jd4) as b2GearJoint;
			
			var jd5:b2GearJointDef = new b2GearJointDef();
			jd5.body1 = body2;
			jd5.body2 = body3;
			jd5.joint1 = m_joint2;
			jd5.joint2 = m_joint3;
			jd5.ratio = -1.0 / circle2.radius;
			m_joint5 = m_world.CreateJoint(jd5) as b2GearJoint;
			
		}
		
		
		//======================
		// Member Data 
		//======================
		public var m_joint1:b2RevoluteJoint;
		public var m_joint2:b2RevoluteJoint;
		public var m_joint3:b2PrismaticJoint;
		public var m_joint4:b2GearJoint;
		public var m_joint5:b2GearJoint;
		
	}
	
}