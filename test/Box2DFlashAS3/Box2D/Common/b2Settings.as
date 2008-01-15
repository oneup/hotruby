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

package Box2D.Common{
	
	
import Box2D.Common.Math.*
	
	
public class b2Settings{

	static public const USHRT_MAX:int = 0x0000ffff;

	static public const b2_pi:Number = Math.PI;
	
	// Define your unit system here. The default system is
	// meters-kilograms-seconds. For the tuning to work well,
	// your dynamic objects should be bigger than a pebble and smaller
	// than a house.
	//static public const b2_lengthUnitsPerMeter:Number = 1.0;
	static public const b2_massUnitsPerKilogram:Number = 1.0;
	static public const b2_timeUnitsPerSecond:Number = 1.0;
	
	// Use this for pixels:
	static public const b2_lengthUnitsPerMeter:Number = 30.0;

	// Global tuning constants based on MKS units.

	// Collision
	static public const b2_maxManifoldPoints:int = 2;
	static public const b2_maxShapesPerBody:int = 64;
	static public const b2_maxPolyVertices:int = 8;
	static public const b2_maxProxies:int = 1024;				// this must be a power of two
	static public const b2_maxPairs:int = 8 * b2_maxProxies;	// this must be a power of two

	// Dynamics
	static public const b2_linearSlop:Number = 0.005 * b2_lengthUnitsPerMeter;	// 0.5 cm
	static public const b2_angularSlop:Number = 2.0 / 180.0 * b2_pi;			// 2 degrees
	static public const b2_velocityThreshold:Number = 1.0 * b2_lengthUnitsPerMeter / b2_timeUnitsPerSecond;		// 1 m/s
	static public const b2_maxLinearCorrection:Number = 0.2 * b2_lengthUnitsPerMeter;	// 20 cm
	static public const b2_maxAngularCorrection:Number = 8.0 / 180.0 * b2_pi;			// 8 degrees
	static public const b2_contactBaumgarte:Number = 0.2;

	// Sleep
	static public const b2_timeToSleep:Number = 0.5 * b2_timeUnitsPerSecond;	// half a second
	static public const b2_linearSleepTolerance:Number = 0.01 * b2_lengthUnitsPerMeter / b2_timeUnitsPerSecond;	// 1 cm/s
	static public const b2_angularSleepTolerance:Number = 2.0 / 180.0 / b2_timeUnitsPerSecond;					// 2 degrees/s
	
	// assert
	static public function b2Assert(a:Boolean) : void
	{
		if (!a){
			var nullVec:b2Vec2;
			nullVec.x++;
		}
	}
}

}