#ifndef UTILITY_FUNCTIONS_H
#define UTILITY_FUNCTIONS_H

#include <glm\glm.hpp>
#include <glm\gtc\matrix_transform.hpp>
#include <glm\gtc\type_ptr.hpp>

#include <vector>

#include "ABary.h"

class UtilityFunctions
{
public:
	static int GetIndexFromIntList(const std::vector<int>& aList, int elem);
	static float DistanceToClosedCurve(std::vector<glm::vec2> polyline, glm::vec2 p);
	static float DistanceToPolyline(const std::vector<glm::vec2>& polyline, glm::vec2 p);
	static float DistanceToFiniteLine(glm::vec2 a, glm::vec2 b, glm::vec2 c);

	static bool InsidePolygon(const std::vector<glm::vec2>& polygon, float px, float py);

	static glm::vec2 GetClosestPtOnClosedCurve(const std::vector<glm::vec2>& polyline, const glm::vec2& p);
	static glm::vec2 ClosestPtAtFiniteLine2(const glm::vec2& v, const glm::vec2& w, const glm::vec2& p); // pbourke algorithm

	static float Angle2D(float x1, float y1, float x2, float y2);
	
	static ABary Barycentric(glm::vec2 p, glm::vec2 A, glm::vec2 B, glm::vec2 C);

	// uniform resample
	static void UniformResampleClosed(std::vector<glm::vec2> oriCurve, std::vector<glm::vec2>& resampleCurve, float resampleGap);
	static void UniformResampleClosed(std::vector<glm::vec2> oriCurve, std::vector<glm::vec2>& resampleCurve, int N);
	static void UniformResample(std::vector<glm::vec2> oriCurve, std::vector<glm::vec2>& resampleCurve, int N);
	static float CurveLengthClosed(std::vector<glm::vec2> curves);
	static float CurveLength(std::vector<glm::vec2> curves);

};

#endif