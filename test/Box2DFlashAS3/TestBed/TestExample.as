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
	
	
	
	public class TestExample extends Test{
		
		public function TestExample(){
			
			// Set Text field
			Main.m_aboutText.text = "Example";
			
			var ground:b2Body = m_world.m_groundBody;
			var bd:b2BodyDef;
			var body:b2Body;
			var circ:b2CircleDef = new b2CircleDef();
			var box:b2BoxDef = new b2BoxDef();
			var jd:b2RevoluteJointDef
			var i:int;
			var angle:Number;
			var tX:Number;
			var tY:Number;
			
			// Create cradle
			{
				circ.radius = 5 / m_physScale;
				circ.density = 1.0;
				circ.friction = 0.0;
				circ.restitution = 1.0;
				
				bd = new b2BodyDef();
				bd.AddShape(circ);
				
				jd = new b2RevoluteJointDef();
				
				const yy:Number = 75 / m_physScale;
				const L:Number = 35 / m_physScale;
				
				for (i = 0; i < 5; ++i)
				{
					var xx:Number = 75 / m_physScale + circ.radius * (2.05 * i);
					bd.position.Set(xx, yy);
					if (i == 0){
						bd.position.Set(xx - L, yy-L);
					}
					body = m_world.CreateBody(bd);
					
					jd.anchorPoint.Set(xx, yy - L);
					jd.body1 = ground;
					jd.body2 = body;
					m_world.CreateJoint(jd);
				}
			}
			
			// Create plane, ball and dominoes
			{
				// plane
				box.extents.Set(180 / m_physScale, 5 / m_physScale);
				box.friction = 0.45;
				bd = new b2BodyDef();
				bd.AddShape(box);
				bd.position.Set(330 / m_physScale, 70 / m_physScale);
				m_world.CreateBody(bd);
				
				// ball
				circ.radius = 10 / m_physScale;
				circ.density = 1;
				circ.friction = 0.5;
				circ.restitution = 0.0;
				bd = new b2BodyDef();
				bd.AddShape(circ);
				bd.position.Set(160 / m_physScale, 55 / m_physScale);
				m_world.CreateBody(bd);
				
				// dominoes
				box.extents.Set(1.5 / m_physScale, 10 / m_physScale);
				box.friction = 0.3;
				box.density = 1;
				bd = new b2BodyDef();
				bd.AddShape(box);
				for (i = 0; i < 30; i++){
					bd.position.Set((215 + i*10) / m_physScale, 55 / m_physScale);
					body = m_world.CreateBody(bd);
					body.m_flags |= b2Body.e_sleepFlag;
				}
			}
			
			// Box, ramp, and circle
			{
				// box
				box.extents.Set(20 / m_physScale, 15 / m_physScale);
				box.friction = 0.3;
				box.density = 0.0;
				bd = new b2BodyDef();
				bd.AddShape(box);
				bd.position.Set(530 / m_physScale, 150 / m_physScale);
				m_world.CreateBody(bd);
				
				// ramp
				box.extents.Set(200 / m_physScale, 5 / m_physScale);
				box.friction = 0.3;
				box.density = 0.0;
				bd = new b2BodyDef();
				bd.AddShape(box);
				bd.position.Set(340 / m_physScale, 240 / m_physScale);
				bd.rotation = -30 / (180/Math.PI);
				m_world.CreateBody(bd);
				
				// circle
				circ.radius = 15 / m_physScale;
				circ.density = 0.9;
				circ.friction = 0.3;
				circ.restitution = 0.3;
				bd = new b2BodyDef();
				bd.AddShape(circ);
				bd.position.Set(510 / m_physScale, 120 / m_physScale);
				m_world.CreateBody(bd);
				
			}
			
			// Stack for circle to plow through
			{
				// box
				box.extents.Set(7 / m_physScale, 7 / m_physScale);
				box.friction = 1.0;
				box.density = 0.5;
				bd = new b2BodyDef();
				bd.AddShape(box);
				for (var x:int = 0; x < 3; x++){
					for (var y:int = 0; y < 15; y++){
						bd.position.Set((50 + x * 16) / m_physScale, (348 - y * 13.5) / m_physScale);
						body = m_world.CreateBody(bd);
						body.m_flags |= b2Body.e_sleepFlag;
					}
				}
			}
			
			// Wheel with car
			{
				// wheel
				circ.radius = 10 / m_physScale;
				circ.density = 1;
				circ.friction = 0.5;
				circ.restitution = 0.0;
				bd = new b2BodyDef();
				bd.AddShape(circ);
				jd = new b2RevoluteJointDef();
				for (i = 0; i < 16; i++){
					angle = i/16 * Math.PI * 2;
					tX = 520 + Math.sin(angle) * 52;
					tY = 260 + Math.cos(angle) * 52;
					bd.position.Set(tX / m_physScale, tY / m_physScale);
					body = m_world.CreateBody(bd);
					
					jd.anchorPoint.Set(520 / m_physScale, 260 / m_physScale);
					jd.body1 = ground;
					jd.body2 = body;
					m_world.CreateJoint(jd);
				}
				
				// Car
				//
				// body
				box.extents.Set(15 / m_physScale, 5 / m_physScale);
				box.friction = 0.3;
				box.density = 1.0;
				bd = new b2BodyDef();
				bd.AddShape(box);
				bd.position.Set(520 / m_physScale, 260 / m_physScale);
				var tBody:b2Body = m_world.CreateBody(bd);
				// wheels
				circ.radius = 12 / m_physScale;
				circ.density = 1;
				circ.friction = 0.8;
				circ.restitution = 0.0;
				bd = new b2BodyDef();
				bd.AddShape(circ);
				//1
				bd.position.Set(505 / m_physScale, 260 / m_physScale);
				body = m_world.CreateBody(bd);
				jd.anchorPoint.Set(505 / m_physScale, 260 / m_physScale);
				jd.enableMotor = true;
				jd.motorSpeed = -7;
				jd.motorTorque = 10000000;
				jd.body1 = tBody;
				jd.body2 = body;
				m_world.CreateJoint(jd);
				//2
				bd.position.Set(535 / m_physScale, 260 / m_physScale);
				body = m_world.CreateBody(bd);
				jd.anchorPoint.Set(535 / m_physScale, 260 / m_physScale);
				jd.body1 = tBody;
				jd.body2 = body;
				m_world.CreateJoint(jd);
				// tracks
				box.extents.Set(5 / m_physScale, 2 / m_physScale);
				box.friction = 1.0;
				box.density = 1;
				bd = new b2BodyDef();
				bd.AddShape(box);
				jd = new b2RevoluteJointDef();
				jd.enableLimit = true;
				jd.lowerAngle = -20 / (180/Math.PI);
				jd.upperAngle = 20 / (180/Math.PI);
				var firstBody:b2Body;
				for (i = 0; i < 16; i++){
					angle = i/16 * Math.PI * 2;
					tX = 520 + Math.sin(angle) * 22;
					tY = 260 + Math.cos(angle) * 22;
					bd.position.Set(tX / m_physScale, tY / m_physScale);
					bd.rotation = -angle;
					body = m_world.CreateBody(bd);
					
					if (i != 0){
						jd.anchorPoint.Set(((tX / m_physScale) + tBody.m_position.x) / 2, (tY / m_physScale + tBody.m_position.y) / 2);
						jd.body1 = tBody;
						jd.body2 = body;
						m_world.CreateJoint(jd);
					}
					else{
						firstBody = body;
					}
					tBody = body;
				}
				jd.anchorPoint.Set((firstBody.m_position.x + body.m_position.x) / 2, (firstBody.m_position.y + body.m_position.y) / 2);
				jd.body1 = body;
				jd.body2 = firstBody;
				m_world.CreateJoint(jd);
				
			}
			
		}
		
		
		//======================
		// Member Data 
		//======================
	}
	
}