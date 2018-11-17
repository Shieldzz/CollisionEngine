#ifndef _COLLISION_BOUNCE_H_
#define _COLLISION_BOUNCE_H_

#include "Behavior.h"
#include "PhysicEngine.h"
#include "GlobalVariables.h"
#include "Renderer.h"
#include "World.h"


#define BOUCINESS 0.5f
#define FRICTION 0.1f
#define DAMPING 0.2f

class CSPolygonBounce : public CBehavior
{
private:
	virtual void Update(float frameTime) override
	{
		gVars->pPhysicEngine->Activate(true);

		gVars->pPhysicEngine->ForEachCollision([&](const SCollision& collision)
		{
			if (collision.polyA->density == 0 && collision.polyB->density == 0)
				return;
				
			ImpulseInCaseOfCollision(collision);
		});
	}

	void	ImpulseInCaseOfCollision(const SCollision& collision)
	{
		Vec2 rA = collision.point - collision.polyA->position;
		Vec2 rB = collision.point - collision.polyB->position;
		Vec2 vAi = collision.polyA->speed + (rA ^ collision.polyA->angularVelocity);
		Vec2 vBi = collision.polyB->speed + (rB ^ collision.polyB->angularVelocity);

		/*Vec2 relativeSpeed = collision.polyB->speed - collision.polyA->speed;
		float speedRelativeWithDotNormal = relativeSpeed | collision.normal;
		*/
		float vRel = (vBi - vAi) | collision.normal;

		if (vRel < 0.f)
		{
			collision.polyA->invMass = collision.polyA->density != 0.0f ? (1.0f / collision.polyA->GetMass()) : 0.0f;
			collision.polyB->invMass = collision.polyB->density != 0.0f ? (1.0f / collision.polyB->GetMass()) : 0.0f;

			/// calcule masse des deux cercle
			float sommeInvMass = collision.polyA->invMass + collision.polyB->invMass;

			/// Fix position
			CorrectionPosition(collision, sommeInvMass);

			float J = ImpulseWithNormalCollision(collision, sommeInvMass, vRel, rA, rB);

			ApplyFriction(collision, sommeInvMass, J);
		}
	}

	void	CorrectionPosition(const SCollision& collision, const float& sommeInvMass)
	{
		float damping = 0.2f;
		float percent = 0.01f;
		Vec2 correction = collision.normal *  ((collision.distance - percent) / sommeInvMass) * damping;

		///Fix Positon
		collision.polyA->position -= correction * collision.polyA->invMass;
		collision.polyB->position += correction * collision.polyB->invMass;
	}

	float	ImpulseWithNormalCollision(const SCollision& collision, const float& vRel, const float& sommeInvMass, const Vec2 & rA, const Vec2& rB)
	{
		float momentumA = collision.polyA->GetInvInertiaTensor() * (rA ^ collision.normal);
		float momentumB = collision.polyB->GetInvInertiaTensor() * (rB ^ collision.normal);

		float weightRotA = (rA ^ momentumA) | collision.normal;
		float weightRotB = (rB ^ momentumB) | collision.normal;


		/// coefficient de restitution [0,1]
		float restitution = (BOUCINESS + 1.0f);
		/// collision : elastique / cas général
	//	Vec2 relativeSpeed = collision.polyB->speed - collision.polyA->speed;
	//	float speedRelativeWithDotNormal = relativeSpeed | collision.normal;

		/// calcule de la force d'impulsion avec la normal de collision
		float J = ((-restitution * vRel) / (sommeInvMass + weightRotA + weightRotB));

		/// Calcule de VA+ / Force
		Vec2 forcePolyA = collision.normal * J * collision.polyA->invMass;
		collision.polyA->ApplyForce(collision.point, -forcePolyA);
		collision.polyA->angularVelocity += J * momentumA;

		/// Calcule de VB+ / Force
		Vec2 forcePolyB = collision.normal * J * collision.polyB->invMass;
		collision.polyB->ApplyForce(collision.point, forcePolyB);
		collision.polyB->angularVelocity += J * momentumB;

		return J;
	}

	void	ApplyFriction(const SCollision& collision, const float& sommeInverseMass, const float & colImpulse)
	{
		Vec2 relativeSpeed = collision.polyB->speed - collision.polyA->speed;
		Vec2 tangent = relativeSpeed - (collision.normal * (relativeSpeed | collision.normal));
		tangent.Normalize();

		float vTangent = relativeSpeed | tangent;

		float JFriction = (-vTangent) / (sommeInverseMass);
		JFriction = Clamp(JFriction, -abs(colImpulse) * FRICTION, abs(colImpulse) * FRICTION);

		/// Calcule de VA+
		Vec2 forcePolyA = (tangent * JFriction * collision.polyA->invMass);
		collision.polyA->ApplyForce(collision.point, -forcePolyA);
		
		/// Calcule de VB+ 
		Vec2 forcePolyB = tangent * JFriction * collision.polyB->invMass;
		collision.polyB->ApplyForce(collision.point, forcePolyB);
	}
};

#endif