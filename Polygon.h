#ifndef _POLYGON_H_
#define _POLYGON_H_

#include <GL/glew.h>
#include <vector>
#include <memory>


#include "Maths.h"

class CProjection;



class CPolygon
{
private:
	friend class CWorld;

	CPolygon(size_t index);
public:
	~CPolygon();

	Vec2				position;
	Mat2				rotation;
	std::vector<Vec2>	points;
	AABB				aabb;

	void				Build();
	void				Draw();
	size_t				GetIndex() const;

	float				GetArea() const;

	Vec2				TransformPoint(const Vec2& point) const;
	Vec2				InverseTransformPoint(const Vec2& point) const;

	// if point is outside then returned distance is negative (and doesn't make sense)
	bool				IsPointInside(const Vec2& point) const;



	// If line intersect polygon, colDist is the penetration distance, and colPoint most penetrating point of poly inside the line
	bool				IsLineIntersectingPolygon(const Line& line, Vec2& colPoint, float& colDist) const;
	bool				CheckCollision(const CPolygon& poly, struct SCollision& collision) const;




	void				UpdateAABB();

	float				GetMass() const;
	float				GetInertiaTensor() const;
	float				GetInvInertiaTensor() const;

	Vec2				GetPointVelocity(const Vec2& point) const;

	// Physics
	float				density;
	Vec2				speed;
	float				angularVelocity = 0.0f;
	Vec2				forces;
	float				torques = 0.0f;
	float				invMass = 0.0f;

	void				ApplyForce(const Vec2& localPoint, const Vec2& force);


private:
#pragma region SAT
	Vec2				CalculateNormalSAT(size_t index) const;
	CProjection			ProjectPolygonSAT(const Vec2& normal, bool isOther) const;
	bool				CreateSeparatingAxesSAT(const CPolygon& poly, const int& index, const Vec2& normal, float& overlap, Vec2& axis, Vec2& points) const;
	bool				CreateProjectionNormalsSAT(const CPolygon& poly, SCollision& collision) const;
	CProjection			FindAxisLeastPenetration(const CPolygon& poly, const int& index, const Vec2& normal, float& bestDistance, int& faceIndex) const;
#pragma endregion

#pragma region GJK
	bool	ContainOrigin(std::vector<Vec2>& simplex, Vec2& dir, Vec2& contactPoint)	const;
	Vec2	TripleDotProduct(Vec2 dir1, Vec2 dir2, Vec2 dir3) const;
	Vec2	ProjectPolygonGJK(Vec2 normal)	const;
	Vec2	CreateMinkowskiDifference(const CPolygon& poly, Vec2 dir) const;
	bool	CreateProjectionNormalsGJK(const CPolygon& poly, Vec2& colPoint, Vec2& colNormal, float& colDist) const;
#pragma endregion




	void				CreateBuffers();
	void				BindBuffers();
	void				DestroyBuffers();

	void				BuildLines();

	void				ComputeArea();
	void				RecenterOnCenterOfMass(); // Area must be computed
	void				ComputeLocalInertiaTensor(); // Must be centered on center of mass

	GLuint				m_vertexBufferId;
	size_t				m_index;

	std::vector<Line>	m_lines;

	float				m_signedArea;

	// Physics
	float				m_localInertiaTensor; // don't consider mass
};

typedef std::shared_ptr<CPolygon>	CPolygonPtr;

#endif