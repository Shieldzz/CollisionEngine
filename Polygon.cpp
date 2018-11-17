#include "Polygon.h"
#include <GL/glu.h>

#include "InertiaTensor.h"

#include "PhysicEngine.h"
#include "Collision.h"
#include "Projection.h"

#include "GlobalVariables.h"
#include "Renderer.h"

CPolygon::CPolygon(size_t index)
	: m_vertexBufferId(0), m_index(index), density(0.1f)
{
	
}

CPolygon::~CPolygon()
{
	DestroyBuffers();
}

void CPolygon::Build()
{
	m_lines.clear();

	ComputeArea();
	RecenterOnCenterOfMass();
	ComputeLocalInertiaTensor();

	CreateBuffers();
	BuildLines();
}

void CPolygon::Draw()
{
	// Set transforms (qssuming model view mode is set)
	float transfMat[16] = {	rotation.X.x, rotation.X.y, 0.0f, 0.0f,
							rotation.Y.x, rotation.Y.y, 0.0f, 0.0f,
							0.0f, 0.0f, 0.0f, 1.0f,
							position.x, position.y, -1.0f, 1.0f };
	glPushMatrix();
	glMultMatrixf(transfMat);

	// Draw vertices
	BindBuffers();
	glDrawArrays(GL_LINE_LOOP, 0, points.size());
	glDisableClientState(GL_VERTEX_ARRAY);

	glPopMatrix();
}

size_t	CPolygon::GetIndex() const
{
	return m_index;
}

float	CPolygon::GetArea() const
{
	return fabsf(m_signedArea);
}

Vec2	CPolygon::TransformPoint(const Vec2& point) const
{
	return position + rotation * point;
}

Vec2	CPolygon::InverseTransformPoint(const Vec2& point) const
{
	return rotation.GetInverseOrtho() * (point - position);
}

bool	CPolygon::IsPointInside(const Vec2& point) const
{
	float maxDist = -FLT_MAX;

	for (const Line& line : m_lines)
	{
		Line globalLine = line.Transform(rotation, position);
		float pointDist = globalLine.GetPointDist(point);
		maxDist = Max(maxDist, pointDist);
	}

	return maxDist <= 0.0f;
}

bool	CPolygon::IsLineIntersectingPolygon(const Line& line, Vec2& colPoint, float& colDist) const
{
	//float dist = 0.0f;
	float minDist = FLT_MAX;
	Vec2 minPoint;
	float lastDist = 0.0f;
	bool intersecting = false;

	for (const Vec2& point : points)
	{
		Vec2 globalPoint = TransformPoint(point);
		float dist = line.GetPointDist(globalPoint);
		if (dist < minDist)
		{
			minPoint = globalPoint;
			minDist = dist;
		}

		intersecting = intersecting || (dist != 0.0f && lastDist * dist < 0.0f);
		lastDist = dist;
	}

	if (minDist <= 0.0f)
	{
		colDist = -minDist;
		colPoint = minPoint;
	}
	return (minDist <= 0.0f);
}

bool	CPolygon::CheckCollision(const CPolygon& poly, SCollision& collision) const
{
	//gVars->pRenderer->DrawLine(position, poly.position, 0.f, 0.f, 1.f);
	return CreateProjectionNormalsSAT(poly, collision);
}


#pragma region SAT
Vec2	CPolygon::CalculateNormalSAT(size_t index) const
{
	Vec2 a = TransformPoint(points[index]);
	Vec2 b = TransformPoint(points[(index + 1) % points.size()]);

	Vec2 dirNormalize = b - a;
	dirNormalize.Normalize();

	return dirNormalize.GetNormal();
}

bool	CPolygon::CreateSeparatingAxesSAT(const CPolygon& poly, const int& index, const Vec2& normal, float &overlap, Vec2& axis, Vec2& point) const
{
	CProjection p1 = ProjectPolygonSAT(normal, false);
	CProjection p2 = poly.ProjectPolygonSAT(normal, true);

	if (!p1.Overlap(p2))
		return false;
	else
	{
		float newOverlap = p1.GetOverlap(p2);

		if (newOverlap < overlap)
		{
			axis = normal;
			overlap = newOverlap;
			float max = p1.GetMax();
			float max2 = p2.GetMax();

			if (p1.GetMax() < p2.GetMax())
			{
				point = p1.GetPointMax() - (axis * overlap);
			}
			else
			{
				point = p2.GetPointMax();
				axis = normal * -1;
			}
		}
	}



	return true;
}

bool	CPolygon::CreateProjectionNormalsSAT(const CPolygon& poly, SCollision& collision) const
{
	float overlap = FLT_MAX;
	Vec2 axis = Vec2();
	Vec2 point = Vec2();


	// First Polygon
	for (size_t i = 0; i < points.size(); i++)
	{
		Vec2 normal = CalculateNormalSAT(i);
		if (!CreateSeparatingAxesSAT(poly, i, normal, overlap, axis, point))
			return false;
	}

	// Second Polygon
	for (size_t j = 0; j < poly.points.size(); j++)
	{
		Vec2 normal = poly.CalculateNormalSAT(j);
		if (!CreateSeparatingAxesSAT(poly, j, normal, overlap, axis, point))
			return false;
	}

	collision.normal = axis.Normalized();
	collision.distance = overlap;
	collision.point = point;

	return true;
}

CProjection	CPolygon::ProjectPolygonSAT(const Vec2& normal, bool isOther) const
{
	float min = FLT_MAX;
	float max = -FLT_MAX;

	float bestProjection = -FLT_MAX;

	Vec2 pointMin = TransformPoint(points[0]);
	Vec2 pointMax = TransformPoint(points[0]);

	for (size_t i = 0; i < points.size(); i++)
	{
		Vec2 point = TransformPoint(points[i]);
		float dist = normal | point;

		if (isOther)
		{
			if (i == 0)
			{
				min = dist;
				max = dist;
			}
				
			if (dist < min)
			{
				min = dist;
				pointMin = point;
			}
			if (dist > max)
			{
				max = dist;
				pointMax = point;
			}
		}
		else
		{
			if (dist < min)
			{
				min = dist;
				pointMin = point;
			}
			if (dist > max)
			{
				max = dist;
				pointMax = point;
			}
		}
	}

	return CProjection(min, max, pointMax, pointMin);
}

CProjection CPolygon::FindAxisLeastPenetration(const CPolygon& poly, const int& index, const Vec2& normal, float& bestDistance, int& faceIndex) const
{
	CProjection polyB = poly.ProjectPolygonSAT(-normal, true);
	Vec2 point = TransformPoint(points[index]);
	float distPenetration = normal | (polyB.GetPointMax() - point);

	Vec2 bestPoint = point;

	if (distPenetration > bestDistance)
	{
		//bestDistance = distPenetration;
		faceIndex = index;
		bestPoint = point;
	}

	return CProjection(0, bestDistance, point, point);
}

#pragma endregion


#pragma region GJK

Vec2	CPolygon::ProjectPolygonGJK(Vec2 normal)	const
{
	float maxProjection = -FLT_MAX;
	Vec2 pointMax;

	for (Vec2 point : points)
	{
		Vec2 globalPoint = TransformPoint(point);
		float projection = globalPoint | normal;
		if (projection > maxProjection)
		{
			maxProjection = projection;
			pointMax = globalPoint;
		}
	}
	return pointMax;
}

Vec2	CPolygon::CreateMinkowskiDifference(const CPolygon& poly, Vec2 dir) const
{
	Vec2 projectionMax = ProjectPolygonGJK(dir);
	Vec2 projectionMin = poly.ProjectPolygonGJK(Vec2(-dir.x, -dir.y));

	return (projectionMax - projectionMin);
}

bool	CPolygon::ContainOrigin(std::vector<Vec2>& simplex, Vec2& dir, Vec2& contactPoint)	const
{
	Vec2 a = simplex[simplex.size() - 1];
	Vec2 ao = Vec2(-a.x, -a.y);

	if (simplex.size() > 2)
	{
		Vec2 b = simplex[1];
		Vec2 c = simplex[0];

		Vec2 ab = b - a;
		Vec2 ac = c - a;

		Vec2 abPerp = TripleDotProduct(ac, ab, ab);
		Vec2 acPerp = TripleDotProduct(ab, ac, ac);

		/*gVars->pRenderer->DrawLine(a, b, 0.0f, 1.0f, 0.0f);
		gVars->pRenderer->DrawLine(a, c, 1.0f, 0.0f, 0.0f);
		gVars->pRenderer->DrawLine(c, b, 0.0f, 0.0f, 1.0f);
		*/
		if ((abPerp | ao) > 0)
		{
			simplex.erase(simplex.begin());
			dir = abPerp;
		}
		else
		{
			if ((acPerp | ao) > 0)
			{
				simplex.erase(simplex.begin() + 1);
				dir = acPerp;
			}
			else
			{
				contactPoint = a; // test
				return true;
			}
		}
	}
	else
	{
		Vec2 b = simplex[0];
		Vec2 ab = b - a;
		dir = TripleDotProduct(ab, ao, ab);
	}

	return false;
}

Vec2 CPolygon::TripleDotProduct(Vec2 dir1, Vec2 dir2, Vec2 dir3) const
{
	float dot1 = dir1 | dir3;
	float dot2 = dir2 | dir3;

	return (dir2 * dot1) - (dir1 * dot2);
}

bool	CPolygon::CreateProjectionNormalsGJK(const CPolygon& poly, Vec2& colPoint, Vec2& colNormal, float& colDist) const
{
	std::vector<Vec2> simplex = std::vector<Vec2>();

	float absc = 0.0f;
	float coord = 0.0f;
	for (Vec2 point : points)
	{
		Vec2 pointGlobal = TransformPoint(point);
		absc += pointGlobal.x;
		coord += pointGlobal.y;
	}

	absc /= points.size();
	coord /= points.size();

	Vec2 dir = Vec2(absc, coord);
	dir.Normalize();

	simplex.push_back(CreateMinkowskiDifference(poly, dir));
	dir = Vec2(-dir.x, -dir.y);

	while (true)
	{
		simplex.push_back(CreateMinkowskiDifference(poly, dir));
		if ((simplex[simplex.size() - 1] | dir) <= 0)
			return false;
		else
		{
			if (ContainOrigin(simplex, dir, colPoint))
				return true;
		}
	}
}
#pragma endregion

#pragma region Force

void CPolygon::ApplyForce(const Vec2 & localPoint, const Vec2 & force)
{
	forces += force;
	torques += localPoint ^ force;
}

#pragma endregion

void CPolygon::UpdateAABB()
{
	aabb.Center(position);
	for (const Vec2& point : points)
	{
		aabb.Extend(TransformPoint(point));
	}

	#pragma region Debug DrawLine
	/*Vec2 p1 = aabb.min;
	p1.y = aabb.max.y;

	Vec2 p2 = aabb.min;
	p2.x = aabb.max.x;

	gVars->pRenderer->DrawLine(aabb.min, p1, 1.0f, 0.0f, 0.0f);
	gVars->pRenderer->DrawLine(p1, aabb.max, 1.0f, 0.0f, 0.0f);
	gVars->pRenderer->DrawLine(aabb.max, p2, 1.0f, 0.0f, 0.0f);
	gVars->pRenderer->DrawLine(p2, aabb.min, 1.0f, 0.0f, 0.0f);*/
	#pragma endregion
}

float CPolygon::GetMass() const
{
	return density * GetArea();
}

float CPolygon::GetInertiaTensor() const
{
	return m_localInertiaTensor * GetMass();
}

float CPolygon::GetInvInertiaTensor() const
{
	return GetInertiaTensor() != 0.0f ? (1 / GetInertiaTensor()) : 0.0f;
}

Vec2 CPolygon::GetPointVelocity(const Vec2& point) const
{
	return speed + (point - position).GetNormal() * angularVelocity;
}

void CPolygon::CreateBuffers()
{
	DestroyBuffers();

	float* vertices = new float[3 * points.size()];
	for (size_t i = 0; i < points.size(); ++i)
	{
		vertices[3 * i] = points[i].x;
		vertices[3 * i + 1] = points[i].y;
		vertices[3 * i + 2] = 0.0f;
	}

	glGenBuffers(1, &m_vertexBufferId);

	glBindBuffer(GL_ARRAY_BUFFER, m_vertexBufferId);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 3 * points.size(), vertices, GL_STATIC_DRAW);

	glBindBuffer(GL_ARRAY_BUFFER, 0);

	delete[] vertices;
}

void CPolygon::BindBuffers()
{
	if (m_vertexBufferId != 0)
	{
		glBindBuffer(GL_ARRAY_BUFFER, m_vertexBufferId);

		glEnableClientState(GL_VERTEX_ARRAY);
		glVertexPointer(3, GL_FLOAT, 0, (void*)0);
	}
}


void CPolygon::DestroyBuffers()
{
	if (m_vertexBufferId != 0)
	{
		glDeleteBuffers(1, &m_vertexBufferId);
		m_vertexBufferId = 0;
	}
}

void CPolygon::BuildLines()
{
	for (size_t index = 0; index < points.size(); ++index)
	{
		const Vec2& pointA = points[index];
		const Vec2& pointB = points[(index + 1) % points.size()];

		Vec2 lineDir = (pointA - pointB).Normalized();

		m_lines.push_back(Line(pointB, lineDir, (pointA - pointB).GetLength()));
	}
}

void CPolygon::ComputeArea()
{
	m_signedArea = 0.0f;
	for (size_t index = 0; index < points.size(); ++index)
	{
		const Vec2& pointA = points[index];
		const Vec2& pointB = points[(index + 1) % points.size()];
		m_signedArea += pointA.x * pointB.y - pointB.x * pointA.y;
	}
	m_signedArea *= 0.5f;
}

void CPolygon::RecenterOnCenterOfMass()
{
	Vec2 centroid;
	for (size_t index = 0; index < points.size(); ++index)
	{
		const Vec2& pointA = points[index];
		const Vec2& pointB = points[(index + 1) % points.size()];
		float factor = pointA.x * pointB.y - pointB.x * pointA.y;
		centroid.x += (pointA.x + pointB.x) * factor;
		centroid.y += (pointA.y + pointB.y) * factor;
	}
	centroid /= 6.0f * m_signedArea;

	for (Vec2& point : points)
	{
		point -= centroid;
	}
	position += centroid;
}

void CPolygon::ComputeLocalInertiaTensor()
{
	m_localInertiaTensor = 0.0f;
	for (size_t i = 0; i + 1 < points.size(); ++i)
	{
		const Vec2& pointA = points[i];
		const Vec2& pointB = points[i + 1];

		m_localInertiaTensor += ComputeInertiaTensor_Triangle(Vec2(), pointA, pointB);
	}
}


