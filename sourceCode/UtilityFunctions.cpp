#include "UtilityFunctions.h"
#include <cmath>

#define PI 3.14159265359
#define PI2 6.28318530718

int UtilityFunctions::GetIndexFromIntList(const std::vector<int>& aList, int elem)
{
	for (unsigned int a = 0; a < aList.size(); a++)
	{
		if (elem == aList[a]) { return a; }
	}

	return -1;
}

float UtilityFunctions::DistanceToClosedCurve(std::vector<glm::vec2> polyline, glm::vec2 p)
{
	polyline.push_back(polyline[0]);
	return UtilityFunctions::DistanceToPolyline(polyline, p);
}

float UtilityFunctions::DistanceToPolyline(const std::vector<glm::vec2>& polyline, glm::vec2 p)
{
	float dist = std::numeric_limits<float>::max();
	for (unsigned int a = 1; a < polyline.size(); a++)
	{
		float d = DistanceToFiniteLine(polyline[a - 1], polyline[a], p);
		if (d < dist) { dist = d; }
	}
	return dist;
}

float UtilityFunctions::DistanceToFiniteLine(glm::vec2 a, glm::vec2 b, glm::vec2 c)
{
	glm::vec2 ab = b - a;
	// Project c onto ab, but deferring divide by Dot(ab, ab)
	//float t = Dot(c - a, ab);
	float t = glm::dot((c - a), ab);
	if (t <= 0.0f)
	{
		// c projects outside the [a,b] interval, on the a side; clamp to a
		//t = 0.0f;
		//d = a;
		return glm::distance(c, a);
	}
	else
	{
		//float denom = Dot(ab, ab); // Always nonnegative since denom = ||ab|| ^ 2
		float denom = glm::dot(ab, ab);
		if (t >= denom)
		{
			// c projects outside the [a,b] interval, on the b side; clamp to b
			//t = 1.0f;
			//d = b;
			return glm::distance(c, b);
		}
		else {
			// c projects inside the [a,b] interval; must do deferred divide now
			t = t / denom;
			//d = a + t * ab;
			glm::vec2 d = a + ab * t;
			return glm::distance(c, d);
		}
	}
}

bool UtilityFunctions::InsidePolygon(const std::vector<glm::vec2>& polygon, float px, float py)
{
	// http_:_//alienryderflex_._com/polygon/
	int poly_sz = polygon.size();

	bool oddNodes = false;

	unsigned int i;
	unsigned int j = poly_sz - 1;
	for (i = 0; i < poly_sz; i++)
	{
		if ((polygon[i].y < py && polygon[j].y >= py ||
			polygon[j].y < py && polygon[i].y >= py)
			&& (polygon[i].x <= px || polygon[j].x <= px))
		{
			oddNodes ^= (polygon[i].x + (py - polygon[i].y) / (polygon[j].y - polygon[i].y) * (polygon[j].x - polygon[i].x) < px);
		}
		j = i;
	}

	return oddNodes;
	
}

inline float DistSquared(const glm::vec2 &p, const glm::vec2 &other)
{
	float xDist = p.x - other.x;
	float yDist = p.y - other.y;
	return xDist * xDist + yDist * yDist;
}


glm::vec2 UtilityFunctions::GetClosestPtOnClosedCurve(const std::vector<glm::vec2>& polyline, const glm::vec2 &p)
{
	float dist = 10000000000;
	glm::vec2 closestPt;
	glm::vec2 pt;
	float d;
	int p_size = polyline.size();
	for (unsigned int a = 1; a < p_size; a++)
	{
		pt = ClosestPtAtFiniteLine2(polyline[a - 1], polyline[a], p);
		d = DistSquared(p, pt); // p.DistanceSquared(pt);
		if (d < dist)
		{
			dist = d;
			closestPt = pt;
		}
	}

	 // first and last point
	pt = ClosestPtAtFiniteLine2(polyline[p_size - 1], polyline[0], p);
	d = DistSquared(p, pt); //p.DistanceSquared(pt);
	if (d < dist)
	{
		dist = d;
		closestPt = pt;
	}
	

	return closestPt;
}

glm::vec2 UtilityFunctions::ClosestPtAtFiniteLine2(const glm::vec2& lnStart, const glm::vec2& lnEnd, const glm::vec2& pt)
{
	float dx = lnEnd.x - lnStart.x;
	float dy = lnEnd.y - lnStart.y;

	float lineMagSq = DistSquared(lnStart, lnEnd); //lnStart.DistanceSquared(lnEnd);

	float u = (((pt.x - lnStart.x) * dx) +
		((pt.y - lnStart.y) * dy)) /
		lineMagSq;

	if (u < 0.0f) { return lnStart; }
	else if (u > 1.0f) { return lnEnd; }

	return glm::vec2(lnStart.x + u * dx, lnStart.y + u * dy);

	
}

float UtilityFunctions::Angle2D(float x1, float y1, float x2, float y2)
{
	float dtheta, theta1, theta2;
	
	theta1 = atan2(y1, x1);
	theta2 = atan2(y2, x2);
	dtheta = theta2 - theta1;

	while (dtheta > PI)
	{
		dtheta -= PI2;
	}

	while (dtheta < -PI)
	{
		dtheta += PI2;
	}

	return dtheta;
}

ABary UtilityFunctions::Barycentric(glm::vec2 p, glm::vec2 A, glm::vec2 B, glm::vec2 C)
{
	ABary bary;

	glm::vec2 v0 = B - A;
	glm::vec2 v1 = C - A;
	glm::vec2 v2 = p - A;

	float d00	= glm::dot(v0, v0);
	float d01	= glm::dot(v0, v1);
	float d11	= glm::dot(v1, v1);
	float d20	= glm::dot(v2, v0);
	float d21	= glm::dot(v2, v1);
	float denom = d00 * d11 - d01 * d01;
	bary._v		= (d11 * d20 - d01 * d21) / denom;
	bary._w		= (d00 * d21 - d01 * d20) / denom;
	bary._u		= 1.0 - bary._v - bary._w;

	return bary;
}

float UtilityFunctions::CurveLengthClosed(std::vector<glm::vec2> curves)
{
	curves.push_back(curves[curves.size() - 1]);
	return CurveLength(curves);
}

float UtilityFunctions::CurveLength(std::vector<glm::vec2> curves)
{
	float length = 0.0;
	for (size_t a = 1; a < curves.size(); a++) 
	{ 
		length += glm::distance(curves[a], curves[a - 1]);
	}
	return length;
}

void UtilityFunctions::UniformResampleClosed(std::vector<glm::vec2> oriCurve, std::vector<glm::vec2>& resampleCurve, float resampleGap)
{
	float l = CurveLengthClosed(oriCurve);
	int N = l / resampleGap;
	if (N < 20)
	{
		resampleCurve = oriCurve;
	}
	else
	{
		UniformResampleClosed(oriCurve, resampleCurve, N);
	}
}

void UtilityFunctions::UniformResampleClosed(std::vector<glm::vec2> oriCurve, std::vector<glm::vec2>& resampleCurve, int N)
{
	glm::vec2 startPt = oriCurve[0];
	glm::vec2 endPt = oriCurve[oriCurve.size() - 1];
	if (glm::distance(startPt, endPt)  > 0.01)
	{
		oriCurve.push_back(oriCurve[0]);
	}

	return UtilityFunctions::UniformResample(oriCurve, resampleCurve, N);
}

void UtilityFunctions::UniformResample(std::vector<glm::vec2> oriCurve, std::vector<glm::vec2>& resampleCurve, int N)
{
	resampleCurve.clear();
	float curveLength = CurveLength(oriCurve);
	float resampleLength = curveLength / (float)(N - 1);

	//int i = 0;
	int iter = 0;
	float sumDist = 0.0;
	resampleCurve.push_back(oriCurve[0]);
	while (resampleCurve.size() < (N - 1))
	{
		float currentDist = glm::distance(oriCurve[iter], oriCurve[iter + 1]); 
		sumDist += currentDist;

		if (sumDist > resampleLength)
		{
			float vectorLength = currentDist - (sumDist - resampleLength);
			glm::vec2 pt1 = oriCurve[iter];
			glm::vec2 pt2 = oriCurve[iter + 1];
			glm::vec2 directionVector = glm::normalize(pt2 - pt1);

			glm::vec2 newPoint1 = pt1 + directionVector * vectorLength;
			resampleCurve.push_back(newPoint1);

			sumDist = currentDist - vectorLength;

			while (sumDist - resampleLength > 1e-8)
			{
				glm::vec2 insertPt2 = resampleCurve[resampleCurve.size() - 1] + directionVector * resampleLength;
				resampleCurve.push_back(insertPt2);
				sumDist -= resampleLength;
			}
		}
		iter++;
	}
	resampleCurve.push_back(oriCurve[oriCurve.size() - 1]); // N - 1

	// bug 
	if (resampleCurve.size() > N)
	{
		resampleCurve.pop_back();
	}
}

