#include "bulletcollisionnotifier.h"
#include "oxygen\physicsserver\collider.h"
#include "boost/shared_ptr.hpp"

using namespace oxygen;
BulletCollisionNotifier::BulletCollisionNotifier(void)
{
}


BulletCollisionNotifier::~BulletCollisionNotifier(void)
{
}

void 	BulletCollisionNotifier::updateAction (btCollisionWorld *collisionWorld, btScalar deltaTimeStep){
    //collisions.clear();


	std::cerr << "(BulletCollisionNotifier) ERROR called un-finished updateAction()" << std::endl;

	int numManifolds = collisionWorld->getDispatcher()->getNumManifolds();
    for (int i=0;i<numManifolds;i++)
    {
        btPersistentManifold* contactManifold =  collisionWorld->getDispatcher()->getManifoldByIndexInternal(i);
        btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
        btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());

        boost::shared_ptr<Collider> collider = Collider::GetCollider((long)obA);
        boost::shared_ptr<Collider> collidee = Collider::GetCollider((long)obB);

        if (
            (collider.get() == 0) ||
            (collidee.get() == 0)
            )
        {
            return;
        }
        //TODO: Check for spaces and internal collision flag or implement space collisions
        // in another way (callback check in broadphase)
        /////////////////////////////////////////////////////////////
        //if (s1 == s2)
        //{
        //    const oxygen::Collider::TColliderNameSet& collider_set = collider->GetNotCollideWithSet();
        //    const oxygen::Collider::TColliderNameSet& collidee_set = collidee->GetNotCollideWithSet();
        //    if (
        //        (collider_set.find(collidee->GetName()) != collider_set.end()) ||
        //        (collidee_set.find(collider->GetName()) != collidee_set.end())
        //        )
        //        {
        //            return;
        //       }
        //}
        //
        //mSpaceImp->CollideInternal(collider, collidee, obj1, obj2);
        //////////////////////////////////////////////////////////////7

        int numContacts = contactManifold->getNumContacts();
        for (int j=0;j<numContacts;j++)
        {
            btManifoldPoint& pt = contactManifold->getContactPoint(j);
            if (pt.getDistance()<0.f)
            {
                const btVector3& ptA = pt.getPositionWorldOnA();
                const btVector3& ptB = pt.getPositionWorldOnB();
                const btVector3& normalOnB = pt.m_normalWorldOnB;
                //btVector3FloatData dataA;
                //btVector3FloatData dataB;
                //ptA.serialize(dataA);
                //ptB.serialize(dataB);
                //collisions.push_back(dataA);
                //collisions.push_back(dataB);
                collider->OnCollision(collidee,(GenericContact&) ptA,Collider::CT_DIRECT);
                collidee->OnCollision(collider,(GenericContact&) ptB,Collider::CT_SYMMETRIC);

            }
        }
    }
}
void 	BulletCollisionNotifier::debugDraw (btIDebugDraw *debugDrawer){

    //Debug drawer not used in spark but might be opened seperately
    //if so then uncommenting the lines that cache the collisions is
    //necessary

    //btVector3 color(btScalar(1.0f),btScalar(0.0f),btScalar(0.0f));
    //btVector3 boxsize(btScalar(0.05f),btScalar(0.05f),btScalar(0.05f));
    //btVector3 pos;

    //auto iter = collisions.begin();
    //while(iter != collisions.end()){
    //    pos.deSerialize(*iter);
    //    debugDrawer->drawBox(pos-boxsize,pos+boxsize,btTransform::getIdentity(),color);
    //    iter++;
    //}
}


