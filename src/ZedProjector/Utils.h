/***********************************************************************
Utils - Various spatial methods.
Copyright (c) 2016 Thomas Wolf

--- Adapted from ofxCSGUtils by lars berg:
https://github.com/larsberg/ofxCSG
Copyright (c) 2015 lars berg

This file is part of the Magic Sand.

The Magic Sand is free software; you can redistribute it
and/or modify it under the terms of the GNU General Public License as
published by the Free Software Foundation; either version 2 of the
License, or (at your option) any later version.

The Magic Sand is distributed in the hope that it will be
useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License along
with the Magic Sand; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA


***********************************************************************/

#pragma once

#include "ofMain.h"

typedef glm::vec3 ofGlmPoint;


namespace ofxCSG
{
	//STATIC VARS
	static float EPSILON = 1e-5;
	static float ONE_PLUS_EPSILON = EPSILON + 1;
	static float NEG_EPSILON = -EPSILON;
	
	enum Classification
	{
		UNDEFINED = 0,
		SPANNING = 1,
		FRONT = 2,
		BACK = 3,
		COPLANAR = 4
	};

	//STATIC METHODS
	template<class T>
	static T lerp(T a, T b, float k)
	{
		return a + (b - a) * k;
	}
	
	template<class T>
	static void appendVectors( vector<T>& a, vector<T>& b )
	{
		//a.reserve( a.size() + b.size() );
		a.insert( a.end(), b.begin(), b.end() );
	}
	
	static glm::vec3 normalFromPoints(glm::vec3 p0, glm::vec3 p1, glm::vec3 p2)
	{
		glm::vec3 result = glm::cross((p2 - p1), (p0 - p1));
		return glm::normalize(result);
	}
	
	static float areaOfTriangle(glm::vec3 p0, glm::vec3 p1, glm::vec3 p2)
	{
		glm::vec3 result = glm::cross((p2 - p1), (p0 - p1));
		return result.length() * .5;
	}
	
	static float areaOfTriangleSquared(glm::vec3 p0, glm::vec3 p1, glm::vec3 p2)
	{
		glm::vec3 result = glm::cross((p2 - p1), (p0 - p1));
		return glm::length2(result) * .5;
	}

	static float signedDistanceToPlane(glm::vec3 point, glm::vec3 planePos, glm::vec3 planeNormal)
	{
		float dist =glm::dot(planeNormal,point - planePos);
		return dist;
	}
	
	//http://geomalgorithms.com/a04-_planes.html
	static float distanceToPlane(glm::vec3 point, glm::vec3 planePos, glm::vec3 planeNormal)
	{
		float sb, sn, sd;
		
		sn = -(glm::dot(planeNormal,point - planePos) );
		sd = glm::dot(planeNormal,planeNormal);
		sb = sn / sd;
		
		glm::vec3 B = point + sb * planeNormal;
		
		return glm::distance(point,B);
	}
	
	static float distanceToPlaneSigned(glm::vec3 point, glm::vec3 planePos, glm::vec3 planeNormal)
	{
		//assumes planeNormal is a unit vector
		return -(glm::dot(planeNormal, point - planePos ) );
		//	return -( doubleDot( planeNormal, point - planePos ) );
	}
	
	static Classification classifyPointWithPlane( glm::vec3 point, glm::vec3 planeNormal, float w )
	{
		float t = glm::dot(planeNormal, point ) - w;
		return ( t < NEG_EPSILON ) ? BACK : (t > EPSILON) ? FRONT : SPANNING;
	}
	
	static Classification classifyPointWithPlane( glm::vec3 point, glm::vec3 planePos, glm::vec3 planeNormal)
	{
		auto d = distanceToPlaneSigned( point, planePos, planeNormal );
		
		if( d > EPSILON )	return BACK;
		else if( d < NEG_EPSILON )	return FRONT;
		
		return SPANNING;
	}
	
	//barycentric coords
	//http://www.blackpawn.com/texts/pointinpoly/
	static bool getBaryCentricCoords(glm::vec3 p, glm::vec3 t0, glm::vec3 t1, glm::vec3 t2, float &u, float &v, float& w)
	{
		// Compute vectors
		glm::vec3 v0 = t2 - t0;
		glm::vec3 v1 = t1 - t0;
		glm::vec3 v2 = p - t0;
		
		// Compute dot products
		float dot00 = glm::dot(v0, v0 );
		float dot01 = glm::dot(v0, v1 );
		float dot02 = glm::dot(v0, v2 );
		float dot11 = glm::dot(v1, v1 );
		float dot12 = glm::dot(v1, v2 );
		
		float denom = (dot00 * dot11 - dot01 * dot01);
		
		if ( denom == 0 )
		{
			//TODO: what's the right thing to do here?
			u = v = w = 0;
			return false;
		}
		
		// Compute barycentric coordinates
		float invDenom = 1.f / denom;
		u = (dot11 * dot02 - dot01 * dot12) * invDenom;
		v = (dot00 * dot12 - dot01 * dot02) * invDenom;
		w = 1. - u - v;
		
		return true;
	}
	
	static bool getBaryCentricCoords(glm::vec3 p, glm::vec3 t0, glm::vec3 t1, glm::vec3 t2, float &u, float &v)
	{
		float w;
		return getBaryCentricCoords(p, t0, t1, t2, u, v, w);
	}
    
    static glm::vec4 getPlaneEquation(glm::vec3 basePlanePos, glm::vec3 basePlaneNormal){
        glm::vec4 basePlaneEq = glm::vec4(basePlaneNormal/basePlaneNormal.length(),0); // Vecteur normal au plan normalisé
        basePlaneEq.w=-glm::dot(basePlaneNormal,basePlanePos);
        return basePlaneEq;
    }
	
	static glm::vec3 closestPointOnLineSegment(glm::vec3 p, glm::vec3 l0, glm::vec3 l1)
	{
		glm::vec3 diff = p - l0;
		glm::vec3 dir = l1 - l0;
		float u = glm::dot(diff, dir ) / glm::dot(dir, dir );
		
		if ( u < 0. )	return l0;
		else if( u > 1. )	return l1;
		
		return l0 + dir * u;
	}
	
	
	//http://paulbourke.net/geometry/pointlineplane/lineline.c
	static bool LineLineIntersect( glm::vec3 p1,glm::vec3 p2,glm::vec3 p3,glm::vec3 p4, glm::vec3 *pa = NULL, glm::vec3 *pb = NULL )
	{
		glm::vec3 p13, p43, p21;
		double d1343,d4321,d1321,d4343,d2121;
		double numer,denom;
		
		p13 = p1 - p3;
		p43 = p4 - p3;
		
		if (abs(p43.x) < EPSILON && abs(p43.y) < EPSILON && abs(p43.z) < EPSILON)	return false;
		
		p21 = p2 - p1;
		
		if (abs(p21.x) < EPSILON && abs(p21.y) < EPSILON && abs(p21.z) < EPSILON)	return false;
		
		d1343 = p13.x * p43.x + p13.y * p43.y + p13.z * p43.z;
		d4321 = p43.x * p21.x + p43.y * p21.y + p43.z * p21.z;
		d1321 = p13.x * p21.x + p13.y * p21.y + p13.z * p21.z;
		d4343 = p43.x * p43.x + p43.y * p43.y + p43.z * p43.z;
		d2121 = p21.x * p21.x + p21.y * p21.y + p21.z * p21.z;
		
		denom = d2121 * d4343 - d4321 * d4321;
		if (abs(denom) < EPSILON)	   return false;
		
		numer = d1343 * d4321 - d1321 * d4343;
		
		double mua = numer / denom;
		double mub = (d1343 + d4321 * mua) / d4343;
		
		if( pa != NULL)
		{
			*pa = p1 + mua * p21;
		}
		if( pb != NULL )
		{
			*pb = p3 + mub * p43;
		}
		
		return true;
	}
	
	static float getLineSegmentUValue(glm::vec3 l0, glm::vec3 l1, glm::vec3 p)
	{
		glm::vec3 diff = p - l0;
		glm::vec3 dir = l1 - l0;
		
		if(l0 == l1)
		{
			return 0;
		}
		
		return glm::dot(diff, dir ) / glm::dot(dir, dir );
	}
	
	static bool isPointInLineSegment(glm::vec3 l0, glm::vec3 l1, glm::vec3 p)
	{
		float u = getLineSegmentUValue( l0, l1, p );
		return  u >= NEG_EPSILON && u <= ONE_PLUS_EPSILON;
	}
	
	static bool intersectLineSegments(glm::vec3 a0, glm::vec3 a1, glm::vec3 b0, glm::vec3 b1, glm::vec3* intersection=NULL)
	{
		glm::vec3 p;
		
		LineLineIntersect(a0, a1, b0, b1, &p);
		
		if( isPointInLineSegment(a0, a1, p) )
		{
			*intersection = p;
			return true;
		}
		
		return false;
	}
	
	static bool splitLineSegmentWithPlane( glm::vec3 l0, glm::vec3 l1, glm::vec3 planeNormal, float w, glm::vec3* intersection)
	{
		auto c0 = classifyPointWithPlane( l0, planeNormal, w);
		auto c1 = classifyPointWithPlane( l1, planeNormal, w);
		
		if( c0 != c1 )
		{
			float k = (w - glm::dot(planeNormal,l0)) / glm::dot(planeNormal, l1 - l0 );
			
			*intersection = glm::lerp( l0, l1, CLAMP(k, 0, 1) ); // the clamp fixed some errors where k > 1
			
			return true;
		}
		
		return false;
	}
	
	static int intersectLineSegmentPlane(glm::vec3 p0, glm::vec3 p1, glm::vec3 planePos, glm::vec3 planeNormal, glm::vec3* intersection = NULL)
	{
		auto d0 = distanceToPlaneSigned( p0, planePos, planeNormal );
		auto d1 = distanceToPlaneSigned( p1, planePos, planeNormal );
		
		if( (d0 >= EPSILON && d1 >= EPSILON) || ( d0 <= NEG_EPSILON && d1 <= NEG_EPSILON ) )
//		if( (d0 > 0 && d1 > 0) || ( d0 < 0 && d1 < 0 ) )
		{
			//no intersection
			return 0;
		}
		if( d0 == 0 && d1 == 0 )
		{
			//it's coplanar
			if( intersection != NULL )
			{
				*intersection = p0;
			}
			return 2;
		}
		
		//it's a hit
		if( intersection != NULL )
		{
			//lerp using the distance to plane values
			*intersection = glm::lerp( p0, p1, d0 / (d0 - d1) );
		}
		return 1;
	}
	
	
	static bool isPointInTriangle(glm::vec3 p, glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 normal )
	{
		if( fabs( distanceToPlaneSigned( p, a, normal ) ) > EPSILON )	return false;
		
		float u, v, w, epsilon = NEG_EPSILON; // 0; // EPSILON; //
		
		if( getBaryCentricCoords( p, a, b, c, u, v, w ) )
		{
			return u > epsilon && v > epsilon && w > epsilon;
		}
		
		return false;
	}
	
	static bool isPointOnPlane( glm::vec3 p, glm::vec3 planeNormal, float w, float epsilon = EPSILON)
	{
		float t = glm::dot(planeNormal,p) - w;
		return abs(t) > epsilon;
	}
	
	static bool isPointInTriangle(glm::vec3 p, glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 normal, float epsilon )
	{
		float u, v, w;
		
		if( getBaryCentricCoords( p, a, b, c, u, v, w ) )
		{
			return u > epsilon && v > epsilon && w > epsilon;
		}
		
		return false;
	}
	
	static bool isPointInTriangle(glm::vec3 p, glm::vec3 a, glm::vec3 b, glm::vec3 c)
	{
		return isPointInTriangle( p, a, b, c, normalFromPoints(a, b, c) );
	}
	
	
	//derived from Akira-Hayasaka's ofxRayTriangleIntersection
	//	https://github.com/Akira-Hayasaka/ofxRayTriangleIntersection/blob/master/src/ofxRayTriangleIntersection.h
	//	assume ray direction is normalized
	static bool intersectRayTriangle(glm::vec3 rayOrigin, glm::vec3 rayDir, glm::vec3 t0, glm::vec3 t1, glm::vec3 t2, glm::vec3* intersection=NULL)
	{
		glm::vec3 normal =glm::normalize(glm::cross((t2 - t1),( t0 - t1)));
		float vn = glm::dot(rayDir,normal);
		
		glm::vec3 diff = rayOrigin - t0;
		float xpn = glm::dot(diff,normal);
		float distance = -xpn / vn;
		
		if (distance < 0) return false; // behind ray origin. fail
		
		glm::vec3 hitPos = rayDir * distance + rayOrigin;
		
		if(isPointInTriangle(hitPos, t0, t1, t2))
		{
			//it's a hit
			if(intersection!= NULL)
			{
				*intersection = hitPos;
			}
			return true;
		}
		
		//nada
		return false;
	}
    
    // Compute plane equation from point cloud
    static glm::vec4 plane_from_points(glm::vec3* points, int n) {
        if (n < 3){
            ofLogVerbose("GreatSand") << "At least three points required" << endl;
            return glm::vec4(glm::vec3(),0);
        }
        
        glm::vec3 sum = glm::vec3(0,0,0);
        for (int i = 0; i < n; i++) {
            sum = sum + points[i];
        }
        glm::vec3 centroid = sum/n;
        
        ofLogVerbose("GreatSand") << "Centroid coordinates : " << centroid << endl;

        // Calc full 3x3 covariance matrix, excluding symmetries:
        float xx = 0.0; float xy = 0.0; float xz = 0.0;
        float yy = 0.0; float yz = 0.0; float zz = 0.0;
        
        for (int i = 0; i < n; i++) {
            glm::vec3 r = points[i] - centroid;
            xx += r.x * r.x;
            xy += r.x * r.y;
            xz += r.x * r.z;
            yy += r.y * r.y;
            yz += r.y * r.z;
            zz += r.z * r.z;
        }
        
        float det_x = yy*zz - yz*yz;
        float det_y = xx*zz - xz*xz;
        float det_z = xx*yy - xy*xy;
        
        float det_max = max(det_x, max(det_y, det_z));
        ofLogVerbose("GreatSand") << "det_max : " << det_max << endl;
        if(det_max == 0.0){
            ofLogVerbose("GreatSand") << "The points don't span a plane" << endl;
            return glm::vec4(glm::vec3(),0);
        }
    
        // Pick path with best conditioning:
        glm::vec3 dir;
        if (det_max == det_x) {
            ofLogVerbose("GreatSand") << "Plane oriented toward x" << endl;
            float a = (xz*yz - xy*zz) / det_x;
            float b = (xy*yz - xz*yy) / det_x;
            dir = glm::vec3(1.0, a, b);
        } else if (det_max == det_y) {
            ofLogVerbose("GreatSand") << "Plane oriented toward y" << endl;
            float a = (yz*xz - xy*zz) / det_y;
            float b = (xy*xz - yz*xx) / det_y;
            dir = glm::vec3(a, 1.0, b);
        } else if (det_max == det_z){
            ofLogVerbose("GreatSand") << "Plane oriented toward z" << endl;
            float a = (yz*xy - xz*yy) / det_z;
            float b = (xz*xy - yz*xx) / det_z;
            dir = glm::vec3(a, b, 1.0);
        } else {
            ofLogVerbose("GreatSand") << "Error treating plane orientation" << endl;
        }
        
        return getPlaneEquation(centroid,dir);
    }
}