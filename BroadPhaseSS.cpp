#include "BroadPhaseSS.h"

#include "GlobalVariables.h"
#include "World.h"
#include "Maths.h"

#include <iostream>
#include <algorithm>


void CBroadPhaseSS::GetCollidingPairsToCheck(std::vector<SPolygonPair>& pairsToCheck) 
{
	m_aabbActiveList.clear();

	SetAABBList();
	SortAABB();
	CheckCollision(pairsToCheck);
}

void CBroadPhaseSS::SetAABBList()
{
	m_aabbList.clear();
	gVars->pWorld->GetPolygons(m_aabbList);
}

void CBroadPhaseSS::SortAABB()
{
	std::sort(m_aabbList.begin() , m_aabbList.end(), Overlapping);
}

void CBroadPhaseSS::CheckCollision(std::vector<SPolygonPair>& pairsToCheck)
{
	m_aabbActiveList.push_back(m_aabbList[0]);

	for (size_t i = 1; i < m_aabbList.size(); i++)
	{
		for (size_t j = 0; j < m_aabbActiveList.size(); j++)
		{
			if (m_aabbActiveList[j]->aabb.max.x < m_aabbList[i]->aabb.min.x)
				m_aabbActiveList.erase(m_aabbActiveList.begin() + j);
			else if (m_aabbActiveList[j]->aabb.IntersectY(m_aabbList[i]->aabb)) //IntersectY(&m_aabbList[i]))
			{
				pairsToCheck.push_back(SPolygonPair(m_aabbActiveList[j], m_aabbList[i]));
			}
		}
		m_aabbActiveList.push_back(m_aabbList[i]);
	}
}

bool CBroadPhaseSS::Overlapping(const CPolygonPtr& poly1, const CPolygonPtr& poly2)
{
	return poly1->aabb.min.x < poly2->aabb.min.x ? true : false;
}
