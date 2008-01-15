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
	
	
	
	public class TestCompound extends Test{
		
		public function TestCompound(){
			
			// Set Text field
			Main.m_aboutText.text = "Compound Shapes";
			
			var bd:b2BodyDef;
			var i:int;
			
			{
				var cd1:b2CircleDef = new b2CircleDef();
				cd1.radius = 15 / m_physScale;
				cd1.localPosition.Set(-15 / m_physScale, 15 / m_physScale);
				cd1.density = 2.0;
				
				var cd2:b2CircleDef = new b2CircleDef();
				cd2.radius = 15 / m_physScale;
				cd2.localPosition.Set(15 / m_physScale, 15 / m_physScale);
				cd2.density = 0.0; // massless
				
				bd = new b2BodyDef();
				
				bd.AddShape(cd1);
				bd.AddShape(cd2);
				
				for (i = 0; i < 4; ++i)
				{
					var r:Number = Math.random()*10 - 5;
					bd.position.Set((100 + r + 640/2) / m_physScale, (100 + i*50) / m_physScale);
					bd.rotation = Math.random() * Math.PI;
					m_world.CreateBody(bd);
				}
			}
			
			{
				var bd1:b2BoxDef = new b2BoxDef();
				bd1.extents.Set(8 / m_physScale, 15 / m_physScale);
				bd1.density = 2.0;
				
				var bd2:b2BoxDef = new b2BoxDef();
				bd2.extents.Set(8 / m_physScale, 15 / m_physScale);
				bd2.localPosition.Set(0.0, -15 / m_physScale);
				bd2.localRotation = 0.5 * Math.PI;
				bd2.density = 2.0;
				
				bd = new b2BodyDef();
				bd.AddShape(bd1);
				bd.AddShape(bd2);
				
				for (i = 0; i < 4; ++i)
				{
					var r2:Number = Math.random()*10 - 5;
					bd.position.Set((r2 - 100 + 640/2) / m_physScale, (100 + i*50) / m_physScale);
					bd.rotation = Math.random() * Math.PI;
					m_world.CreateBody(bd);
				}
			}
			
			{
				var pd1:b2PolyDef = new b2PolyDef();
				pd1.vertexCount = 3;
				pd1.vertices[0].Set(-30 / m_physScale, 0.0);
				pd1.vertices[1].Set(30 / m_physScale, 0.0);
				pd1.vertices[2].Set(0.0, 15 / m_physScale);
				pd1.localRotation = 0.3524 * Math.PI;
				var R1:b2Mat22 = new b2Mat22(pd1.localRotation);
				pd1.localPosition = b2Math.b2MulMV(R1, new b2Vec2(30.0 / m_physScale, 0.0));
				pd1.density = 2.0;
				
				var pd2:b2PolyDef = new b2PolyDef();
				pd2.vertexCount = 3;
				pd2.vertices[0].Set(-30 / m_physScale, 0.0);
				pd2.vertices[1].Set(30 / m_physScale, 0.0);
				pd2.vertices[2].Set(0.0, 15 / m_physScale);
				pd2.localRotation = -0.3524 * Math.PI;
				var R2:b2Mat22 = new b2Mat22(pd2.localRotation);
				pd2.localPosition = b2Math.b2MulMV(R2, new b2Vec2(-30.0 / m_physScale, 0.0));
				pd2.density = 2.0;
				
				bd = new b2BodyDef();
				bd.AddShape(pd1);
				bd.AddShape(pd2);
				
				for (i = 0; i < 4; ++i)
				{
					var r3:Number = Math.random() * 10 - 5;
					bd.position.Set((r3 + 640/2) / m_physScale, (100 + i*50) / m_physScale);
					bd.rotation = 0.0;
					m_world.CreateBody(bd);
				}
			}
			
		}
		
		
		//======================
		// Member Data 
		//======================
	}
	
}