#ifndef _BROAD_PHASE_SS_H_
#define _BROAD_PHASE_SS_H_

#include "BroadPhase.h"

class CBroadPhaseSS : public IBroadPhase
{

public:
	virtual void GetCollidingPairsToCheck(std::vector<SPolygonPair>& pairsToCheck) override;

private:
	void	CheckCollision(std::vector<SPolygonPair>& pairsToCheck);
	void	SetAABBList();
	void	SortAABB();
	static bool	Overlapping(const CPolygonPtr& a, const CPolygonPtr& b);


	std::vector<CPolygonPtr>	m_aabbList;
	std::vector<CPolygonPtr>	m_aabbActiveList;
};

#endif