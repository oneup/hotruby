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
	
	
	
	public class TestRagdoll extends Test{
		
		public function TestRagdoll(){
			
			// Set Text field
			Main.m_aboutText.text = "Ragdolls";
			
			var bd:b2BodyDef;
			var circ:b2CircleDef = new b2CircleDef();
			var box:b2BoxDef = new b2BoxDef();
			var jd:b2RevoluteJointDef = new b2RevoluteJointDef();
			
			// Add 5 ragdolls along the top
			for (var i:int = 0; i < 4; i++){
				var startX:Number = 100 + 145 * i;
				var startY:Number = 20 + Math.random() * 50;
				
				// BODIES
				
				// Head
				circ.radius = 12.5 / m_physScale;
				circ.density = 1.0;
				circ.friction = 0.4;
				circ.restitution = 0.3;
				bd = new b2BodyDef();
				bd.AddShape(circ);
				bd.position.Set(startX / m_physScale, startY / m_physScale);
				var head:b2Body = m_world.CreateBody(bd);
				
				// Torso1
				box.extents.Set(15 / m_physScale, 10 / m_physScale);
				box.density = 1.0;
				box.friction = 0.4;
				box.restitution = 0.1;
				bd = new b2BodyDef();
				bd.AddShape(box);
				bd.position.Set(startX / m_physScale, (startY + 28) / m_physScale);
				var torso1:b2Body = m_world.CreateBody(bd);
				// Torso2
				bd = new b2BodyDef();
				bd.AddShape(box);
				bd.position.Set(startX / m_physScale, (startY + 43) / m_physScale);
				var torso2:b2Body = m_world.CreateBody(bd);
				// Torso3
				bd = new b2BodyDef();
				bd.AddShape(box);
				bd.position.Set(startX / m_physScale, (startY + 58) / m_physScale);
				var torso3:b2Body = m_world.CreateBody(bd);
				
				// UpperArm
				box.extents.Set(18 / m_physScale, 6.5 / m_physScale);
				box.density = 1.0;
				box.friction = 0.4;
				box.restitution = 0.1;
				bd = new b2BodyDef();
				bd.AddShape(box);
				// L
				bd.position.Set((startX - 30) / m_physScale, (startY + 20) / m_physScale);
				var upperArmL:b2Body = m_world.CreateBody(bd);
				// R
				bd.position.Set((startX + 30) / m_physScale, (startY + 20) / m_physScale);
				var upperArmR:b2Body = m_world.CreateBody(bd);
				
				// LowerArm
				box.extents.Set(17 / m_physScale, 6 / m_physScale);
				box.density = 1.0;
				box.friction = 0.4;
				box.restitution = 0.1;
				bd = new b2BodyDef();
				bd.AddShape(box);
				// L
				bd.position.Set((startX - 57) / m_physScale, (startY + 20) / m_physScale);
				var lowerArmL:b2Body = m_world.CreateBody(bd);
				// R
				bd.position.Set((startX + 57) / m_physScale, (startY + 20) / m_physScale);
				var lowerArmR:b2Body = m_world.CreateBody(bd);
				
				// UpperLeg
				box.extents.Set(7.5 / m_physScale, 22 / m_physScale);
				box.density = 1.0;
				box.friction = 0.4;
				box.restitution = 0.1;
				bd = new b2BodyDef();
				bd.AddShape(box);
				// L
				bd.position.Set((startX - 8) / m_physScale, (startY + 85) / m_physScale);
				var upperLegL:b2Body = m_world.CreateBody(bd);
				// R
				bd.position.Set((startX + 8) / m_physScale, (startY + 85) / m_physScale);
				var upperLegR:b2Body = m_world.CreateBody(bd);
				
				// LowerLeg
				box.extents.Set(6 / m_physScale, 20 / m_physScale);
				box.density = 1.0;
				box.friction = 0.4;
				box.restitution = 0.1;
				bd = new b2BodyDef();
				bd.AddShape(box);
				// L
				bd.position.Set((startX - 8) / m_physScale, (startY + 119) / m_physScale);
				var lowerLegL:b2Body = m_world.CreateBody(bd);
				// R
				bd.position.Set((startX + 8) / m_physScale, (startY + 119) / m_physScale);
				var lowerLegR:b2Body = m_world.CreateBody(bd);
				
				
				// JOINTS
				jd.enableLimit = true;
				
				// Head to shoulders
				jd.lowerAngle = -40 / (180/Math.PI);
				jd.upperAngle = 40 / (180/Math.PI);
				jd.anchorPoint.Set(startX / m_physScale, (startY + 15) / m_physScale);
				jd.body1 = torso1;
				jd.body2 = head;
				m_world.CreateJoint(jd);
				
				// Upper arm to shoulders
				// L
				jd.lowerAngle = -85 / (180/Math.PI);
				jd.upperAngle = 130 / (180/Math.PI);
				jd.anchorPoint.Set((startX - 18) / m_physScale, (startY + 20) / m_physScale);
				jd.body1 = torso1;
				jd.body2 = upperArmL;
				m_world.CreateJoint(jd);
				// R
				jd.lowerAngle = -130 / (180/Math.PI);
				jd.upperAngle = 85 / (180/Math.PI);
				jd.anchorPoint.Set((startX + 18) / m_physScale, (startY + 20) / m_physScale);
				jd.body1 = torso1;
				jd.body2 = upperArmR;
				m_world.CreateJoint(jd);
				
				// Lower arm to upper arm
				// L
				jd.lowerAngle = -130 / (180/Math.PI);
				jd.upperAngle = 10 / (180/Math.PI);
				jd.anchorPoint.Set((startX - 45) / m_physScale, (startY + 20) / m_physScale);
				jd.body1 = upperArmL;
				jd.body2 = lowerArmL;
				m_world.CreateJoint(jd);
				// R
				jd.lowerAngle = -10 / (180/Math.PI);
				jd.upperAngle = 130 / (180/Math.PI);
				jd.anchorPoint.Set((startX + 45) / m_physScale, (startY + 20) / m_physScale);
				jd.body1 = upperArmR;
				jd.body2 = lowerArmR;
				m_world.CreateJoint(jd);
				
				// Shoulders/stomach
				jd.lowerAngle = -15 / (180/Math.PI);
				jd.upperAngle = 15 / (180/Math.PI);
				jd.anchorPoint.Set(startX / m_physScale, (startY + 35) / m_physScale);
				jd.body1 = torso1;
				jd.body2 = torso2;
				m_world.CreateJoint(jd);
				// Stomach/hips
				jd.anchorPoint.Set(startX / m_physScale, (startY + 50) / m_physScale);
				jd.body1 = torso2;
				jd.body2 = torso3;
				m_world.CreateJoint(jd);
				
				// Torso to upper leg
				// L
				jd.lowerAngle = -25 / (180/Math.PI);
				jd.upperAngle = 45 / (180/Math.PI);
				jd.anchorPoint.Set((startX - 8) / m_physScale, (startY + 72) / m_physScale);
				jd.body1 = torso3;
				jd.body2 = upperLegL;
				m_world.CreateJoint(jd);
				// R
				jd.lowerAngle = -45 / (180/Math.PI);
				jd.upperAngle = 25 / (180/Math.PI);
				jd.anchorPoint.Set((startX + 8) / m_physScale, (startY + 72) / m_physScale);
				jd.body1 = torso3;
				jd.body2 = upperLegR;
				m_world.CreateJoint(jd);
				
				// Upper leg to lower leg
				// L
				jd.lowerAngle = -25 / (180/Math.PI);
				jd.upperAngle = 115 / (180/Math.PI);
				jd.anchorPoint.Set((startX - 8) / m_physScale, (startY + 107) / m_physScale);
				jd.body1 = upperLegL;
				jd.body2 = lowerLegL;
				m_world.CreateJoint(jd);
				// R
				jd.lowerAngle = -115 / (180/Math.PI);
				jd.upperAngle = 25 / (180/Math.PI);
				jd.anchorPoint.Set((startX + 8) / m_physScale, (startY + 107) / m_physScale);
				jd.body1 = upperLegR;
				jd.body2 = lowerLegR;
				m_world.CreateJoint(jd);
				
			}
			
			
			// Add 10 random circles for them to hit
			for (var j:int = 0; j < 5; j++){
				circ.radius = (Math.random() * 30 + 30) / m_physScale;
				circ.density = 0.0;
				circ.friction = 0.4;
				circ.restitution = 0.3;
				bd = new b2BodyDef();
				bd.AddShape(circ);
				bd.position.Set((Math.random() * 540 + 50) / m_physScale, (Math.random() * 200 + 150) / m_physScale);
				m_world.CreateBody(bd);
			}
			
			
		}
		
		
		//======================
		// Member Data 
		//======================
	}
	
}