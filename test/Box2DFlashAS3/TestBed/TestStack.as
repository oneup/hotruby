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
	
	
	
	public class TestStack extends Test{
		
		public function TestStack(){
			
			// Set Text field
			Main.m_aboutText.text = "Stacked Boxes";
			
			// Add bodies
			var sd:b2BoxDef = new b2BoxDef();
			var bd:b2BodyDef = new b2BodyDef();
			bd.AddShape(sd);
			sd.density = 1.0;
			sd.friction = 0.5;
			
			var i:int;
			for (i = 0; i < 10; i++){
				sd.extents.Set((10) / m_physScale, (10) / m_physScale);
				bd.position.Set((640/2-Math.random()*2 - 1) / m_physScale, (360-5-i*21) / m_physScale);
				m_world.CreateBody(bd);
			}
			for (i = 0; i < 10; i++){
				sd.extents.Set((10) / m_physScale, (10) / m_physScale);
				bd.position.Set((640/2-100+Math.random()* 5 + i) / m_physScale, (360-5-i*21) / m_physScale);
				m_world.CreateBody(bd);
			}
			for (i = 0; i < 10; i++){
				sd.extents.Set((10) / m_physScale, (10) / m_physScale);
				bd.position.Set((640/2+100+Math.random()* 5 - i) / m_physScale, (360-5-i*21) / m_physScale);
				m_world.CreateBody(bd);
			}
			
			
		}
		
		
		//======================
		// Member Data 
		//======================
	}
	
}