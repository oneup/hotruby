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

package Box2D.Dynamics{


import Box2D.Dynamics.*
import Box2D.Dynamics.Joints.*
import Box2D.Dynamics.Contacts.*
import Box2D.Collision.*
import Box2D.Collision.Shapes.*
import Box2D.Common.b2Settings
import Box2D.Common.Math.*


public class b2BodyDef
{
	public function b2BodyDef()
	{
		userData = null;
		for (var i:int = 0; i < b2Settings.b2_maxShapesPerBody; i++){
			shapes[i] = null;
		}
		position = new b2Vec2(0.0, 0.0);
		rotation = 0.0;
		linearVelocity = new b2Vec2(0.0, 0.0);
		angularVelocity = 0.0;
		linearDamping = 0.0;
		angularDamping = 0.0;
		allowSleep = true;
		isSleeping = false;
		preventRotation = false;
	}

	public var userData:*;
	public var shapes:Array = new Array();
	public var position:b2Vec2;
	public var rotation:Number;
	public var linearVelocity:b2Vec2;
	public var angularVelocity:Number;
	public var linearDamping:Number;
	public var angularDamping:Number;
	public var allowSleep:Boolean;
	public var isSleeping:Boolean;
	public var preventRotation:Boolean;

	public function AddShape(shape:b2ShapeDef) : void
	{
		for (var i:int = 0; i < b2Settings.b2_maxShapesPerBody; ++i)
		{
			if (shapes[i] == null)
			{
				shapes[i] = shape;
				break;
			}
		}
	}
};


}