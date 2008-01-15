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



// The pulley joint is connected to two bodies and two fixed ground points.
// The pulley supports a ratio such that:
// length1 + ratio * length2 = constant
// Yes, the force transmitted is scaled by the ratio.
// The pulley also enforces a maximum length limit on both sides. This is
// useful to prevent one side of the pulley hitting the top.

public class b2PulleyJointDef extends b2JointDef
{
	public function b2PulleyJointDef()
	{
		type = b2Joint.e_pulleyJoint;
		groundPoint1.Set(-1.0, 1.0);
		groundPoint2.Set(1.0, 1.0);
		anchorPoint1.Set(-1.0, 0.0);
		anchorPoint2.Set(1.0, 0.0);
		maxLength1 = 0.5 * b2PulleyJoint.b2_minPulleyLength;
		maxLength2 = 0.5 * b2PulleyJoint.b2_minPulleyLength;
		ratio = 1.0;
		collideConnected = true;
	}

	public var groundPoint1:b2Vec2 = new b2Vec2();
	public var groundPoint2:b2Vec2 = new b2Vec2();
	public var anchorPoint1:b2Vec2 = new b2Vec2();
	public var anchorPoint2:b2Vec2 = new b2Vec2();
	public var maxLength1:Number;
	public var maxLength2:Number;
	public var ratio:Number;
};

}