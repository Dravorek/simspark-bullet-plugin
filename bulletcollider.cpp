/* -*- mode: c++; c-basic-offset: 4; indent-tabs-mode: nil -*-

   this file is part of rcssserver3D
   Fri May 9 2003
   Copyright (C) 2003 Koblenz University
   $Id$

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; version 2 of the License.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/

#include "bulletcollider.h"
#include "bulletbody.h"
#include "bulletspace.h"

#include <oxygen/physicsserver/collider.h>
#include <oxygen/physicsserver/world.h>
#include <oxygen/sceneserver/scene.h>


using namespace oxygen;
using namespace salt;
using namespace boost;
using namespace std;




ColliderImp::ColliderImp() : PhysicsObjectImp(), geomID(NULL), userPointer(NULL)
{
}

ColliderImp::~ColliderImp()
{
}

Collider* ColliderImp::GetColliderPointer(){
    btCollisionShape* BulletGeom = (btCollisionShape *) geomID;
    //TODO: not very safe, check if geomID reallly is the right type
	//if(BulletGeom)
	//	return static_cast<Collider*>(BulletGeom->getUserPointer());
	//else 
		return (Collider*)userPointer;
}

void ColliderImp::SetRotation(const Matrix& rot)
{
    btCollisionShape* BulletGeom = (btCollisionShape *) geomID;
    //:TODO: maplookup for every set position? maybe store a flag inside the collider?
	auto it = collidermap.find(BulletGeom);
	if(it!=collidermap.end()){
		btMatrix3x3 BulletMatrix;
		GenericPhysicsMatrix& matrixRef = (GenericPhysicsMatrix&) BulletMatrix; 
		ConvertRotationMatrix(rot, matrixRef);

		btTransform trans = (it->second->obj)->getWorldTransform();
		trans.setBasis(BulletMatrix);
		(it->second->obj)->setWorldTransform(trans);

	}
}

void ColliderImp::SetPosition(const Vector3f& globalPos)
{
    //:TODO: maplookup for every set position? maybe store a flag inside the collider?
    btCollisionShape* BulletGeom = (btCollisionShape *) geomID;
    
	auto it = collidermap.find(BulletGeom);
	if(it!=collidermap.end()){
		btTransform trans = (it->second->obj)->getWorldTransform();
		trans.setOrigin(btVector3(
            btScalar(globalPos[0]),
            btScalar(globalPos[1]),
            btScalar(globalPos[2])));    
		(it->second->obj)->setWorldTransform(trans);
	}
	
}

void ColliderImp::SetLocalPosition(const Vector3f& pos)
{
    //SetPosition(pos,geomID);
	std::cerr << "(bulletimps) ERROR unimplemented ColliderImp::SetLocalPosition() called."
		      << std::endl;
}

Vector3f ColliderImp::GetPosition() const
{
    //btGeom *BulletGeom = (btGeom *) geomID;
    btCollisionShape* BulletGeom = (btCollisionShape *) geomID;
    
	auto it = collidermap.find(BulletGeom);
	if(it!=collidermap.end()){
		btTransform trans = (it->second->obj)->getWorldTransform();
		return Vector3f(
                (float)trans.getOrigin().x(),
                (float)trans.getOrigin().y(),
                (float)trans.getOrigin().z()
            );
	}
	//std::cerr << "(bulletimps) ERROR unimplemented ColliderImp::GetPosition() called."
	//	      << std::endl;
	//return Vector3f(
 //               (float)0.0f,
 //               (float)0.0f,
 //               (float)0.0f
 //           );
}

SpaceInt *ColliderImp::GetParentSpaceID()
{
    //TODO: check how safe returning 0 is
	std::cerr << "(bulletimps) ERROR unimplemented ColliderImp::GetSpaceID() called."
		      << std::endl;
	for(auto it = spaces.begin();it!=spaces.end();it++){
		if((long)it->second == (long)geomID)
			return (SpaceInt *)it->first;
	}
	return (SpaceInt *)1l;
}

bool ColliderImp::Intersect(boost::shared_ptr<Collider> collider)
{
    //TODO: save collisionhandler in World, retrieve world and ask collisionhandler
    // for collision data
    /*
    dGeomID ODEGeom = (dGeomID) geomID;
    dContactGeom contact;

    return dCollide
        (ODEGeom,
         (dGeomID) collider->GetGeomID(),
         1, // ask for at most one collision point 
         &contact,
         sizeof(contact)
         ) > 0;
         */
	std::cerr << "(bulletimps) ERROR unimplemented ColliderImp::Intersect() called."
		      << std::endl;
	return false;
}

void ColliderImp::DestroyGeom()
{
   std::cerr << "(bulletimps) WARNING notFinished ColliderImp::DestroyGeom() called."
			  << std::endl;
	//TODO: check if deleting the internal pointer is necessary
    //delete (Collider *)BulletGeom->getUserPointer() or
    //even removing the collisionobject from the world
    
    
    /*
    boost::shared_ptr<World> wrld;
    boost::shared_ptr<Scene> scene = GetScene();
    if (scene.get() == 0)
    {
        std::cerr << "(BulletObject) ERROR: found no Scene node\n";
    }else{
        boost::shared_ptr<World> worldNode = boost::shared_dynamic_cast<World>
            (scene->GetChildOfClass("World"));
        if (worldNode.get() == 0)
        {
            std::cerr << "(ODEObject) ERROR: found no World node\n";
        }
        wrld = worldNode;
    }
    if(wrld.get()!=0)
    {
         btDynamicsWorld * world = (btDynamicsWorld *)wrld->GetWorldID();
         world->removeCollisionObject(BulletGeom);
    }
    */
    //TODO: elegant deleting in tune with what bulletrigidbody.cpp
//    if(((btRigidBody *)BulletGeom->obj)!=0)
//    {
//        //TODO: REMOVE FROM WORLD and every constraint
//        delete ((btRigidBody *)BulletGeom->obj);
//    }
	btCollisionShape* BulletGeom = (btCollisionShape *) geomID;
    
	auto it = collidermap.find(BulletGeom);
	//part of RigidBody or CollisionObject
	if(it!=collidermap.end()){
		if(it->second->isRigidBody){
			delete BulletGeom;
			//Rigid Body does still exist
			if(it->second->obj){
				//This is the top-level collider
				if(it->second->shp==BulletGeom)
				{
					btEmptyShape *shp = new btEmptyShape();
					it->second->obj->setCollisionShape(shp);
					collidermap.erase(it);
					//:TODO: insert empty collider into collidermap?
					delete BulletGeom;
				//part of a compound collider?
				}else if(it->second->shp->getShapeType()== COMPOUND_SHAPE_PROXYTYPE){//
					btCompoundShape *shp = (btCompoundShape *)it->second->shp;
					shp->removeChildShape(BulletGeom);
					collidermap.erase(it);
					delete BulletGeom;
				}else{
					std::cerr << "ERROR (BulletCollider) called destroy collider in a broken configuration" <<std::endl;
				}
			}
		}else{//Is part of CollisionObject
				//This is the top-level collider
				if(it->second->shp==BulletGeom)
				{
					it->second->wrld->removeCollisionObject(it->second->obj);
				    delete it->second->obj;
					btGeom * ptr = it->second;
					collidermap.erase(it);
					delete BulletGeom;
					delete ptr;
				//part of a compound collider?
				}else if(it->second->shp->getShapeType()== COMPOUND_SHAPE_PROXYTYPE){//
					btCompoundShape *shp = (btCompoundShape *)it->second->shp;
					shp->removeChildShape(BulletGeom);
					collidermap.erase(it);
					delete BulletGeom;
				}else{
					std::cerr << "ERROR (BulletCollider) called destroy collider in a broken configuration" <<std::endl;
				}
		}
	}
}


void ColliderImp::TransformSetGeom(ColliderInt *parentGeomID){
    //:TODO: figure out how to handle parenting in bullet
    /*dGeomID parentODEGeom = (dGeomID) parentGeomID;
    dGeomID ODEGeom = (dGeomID) geomID;
    dGeomTransformSetGeom(parentODEGeom, ODEGeom);*/
	std::cerr << "(bulletimps) ERROR unimplemented ColliderImp::TransformSetGeom() called."
		      << std::endl;
}

void ColliderImp::SetSpace(SpaceInt *spaceID,Collider* collider){
    //dSpaceID ODESpace = (dSpaceID) spaceID;
    //:TODO: add space handling if necessary
    
 //   btGeom *BulletGeom = (btGeom *) geomID;
 //   
 //   if(BulletGeom->shp && 
	//	BulletGeom->shp->getUserPointer() ==0)
	//{
 //       BulletGeom->shp->setUserPointer(collider);
 //   }
	btCollisionShape *shp = (btCollisionShape *)geomID;
	userPointer=collider;
	
	if(!shp)
	{
		std::cerr << "(bulletimps) (COLLIDER) ERROR  DID NOT FIND COLLIDER SHAPE"<< std::endl;
		return;
	}
	shp->setUserPointer(static_cast<ColliderInt *>(this));
	
	for(auto it = spaces.begin();it!=spaces.end();it++){
		if((long)it->second == (long)geomID){
			spaces.erase(it);
			break;
		}
	}
	spaces.insert(std::pair<long,void*>((int)spaceID,(void*)geomID));

	if(spaceID!=nullptr && (long)spaceID!=1 && 
		!static_cast<SpaceImp *>(spaceID)->innercollision){
		auto it = collidermap.find((btCollisionShape *) geomID);
		//part of CollisionObject?
		if(it!=collidermap.end())
		{
			btGeom *temp = it->second;
			if(temp->obj && temp->wrld)
			{
				if(temp->isRigidBody)
				{
					temp->wrld->removeRigidBody(static_cast<btRigidBody*>(temp->obj));
					temp->wrld->addRigidBody(static_cast<btRigidBody*>(temp->obj),4,~4);
					((btRigidBody *)temp->obj)->activate(true);
					((btRigidBody *)temp->obj)->setActivationState(DISABLE_DEACTIVATION);
				}
				else
				{
					temp->wrld->removeCollisionObject(temp->obj);
					temp->wrld->addCollisionObject(temp->obj,4,~4);
				}
			}
		}
	}
}

void ColliderImp::SetBody(BodyInt *bodyID){
    /// TODO : figure out how to handle parenting in bullet
    
	/*dBodyID ODEBody = (dBodyID) bodyID;
    dGeomID ODEGeom = (dGeomID) geomID;
    dGeomSetBody(ODEGeom, ODEBody);*/
	if(!geomID)
	{
		std::cerr << "(bulletimps) (btCollider) tried to add NULL to a rigidbody"<<std::endl;
		return;
	}

	std::cerr << "(bulletimps) DEBUG called ColliderImp::SetBody(body="<<bodyID<<",geom="<<geomID<<") called."
		      << std::endl;
	btCollisionShape* BulletGeom = (btCollisionShape *) geomID;
    auto it = collidermap.find(BulletGeom);
	//part of CollisionObject?
	if(it!=collidermap.end())
	{
		if(!it->second->isRigidBody)
		{
			if(it->second->shp==BulletGeom)
			{
				it->second->wrld->removeCollisionObject(it->second->obj);
				delete it->second->obj;
				delete it->second;
				collidermap.erase(it);
				//TODO: shouldn't happen ?
				//part of a compound collider?
			}else if(it->second->shp->getShapeType()== COMPOUND_SHAPE_PROXYTYPE){//
				btCompoundShape *shp = (btCompoundShape *)it->second->shp;
				shp->removeChildShape(BulletGeom);
				collidermap.erase(it);
			}
		}
	}
	BodyImp *impPtr = static_cast<BodyImp *>(bodyID);
	btGeom *bulletBody = (btGeom *)impPtr->btID;
	bulletBody->wrld->removeRigidBody(((btRigidBody *)bulletBody->obj));
	((btRigidBody *)bulletBody->obj)->setCollisionShape( (btCollisionShape *)geomID);
	btVector3 inertia;
	btScalar mass = ((btRigidBody *)bulletBody->obj)->getInvMass()==0.0f?0.0:1.0/((btRigidBody *)bulletBody->obj)->getInvMass();
	bulletBody->shp=(btCollisionShape *)geomID;
	bulletBody->shp->calculateLocalInertia(mass,inertia);
	((btRigidBody *)bulletBody->obj)->setMassProps(mass,inertia);
	((btRigidBody *)bulletBody->obj)->updateInertiaTensor();
	
	//CHECK WHETHER INNER COLLISION IS ENAVVLED
	SpaceInt * space =GetParentSpaceID();
	if(space != 0 && (long)space !=1 
		&& !static_cast<SpaceImp *>(space)->innercollision)
	{
		bulletBody->wrld->addRigidBody(((btRigidBody *)bulletBody->obj),4,~4);
	
	}
	else
	{
		bulletBody->wrld->addRigidBody(((btRigidBody *)bulletBody->obj));
	}
	collidermap.insert(std::pair<btCollisionShape *,btGeom *>(bulletBody->shp,bulletBody));
	bulletBody->obj->activate(true);
	((btRigidBody *)bulletBody->obj)->setActivationState(DISABLE_DEACTIVATION);

}

void ColliderImp::RemoveFromSpace(SpaceInt *spaceID){
    // TODO add space handling if necessary
    /*dGeomID ODEGeom = (dGeomID) geomID;
    dSpaceID ODESpace = (dSpaceID) spaceID;
    dSpaceRemove(ODESpace, ODEGeom);*/
	std::cerr << "(bulletimps) DEBUG called ColliderImp::RemoveFromSpace(geom="<<geomID<<",space="<<spaceID<<") called."
		      << std::endl;

	auto range = spaces.equal_range((int)spaceID);
	for(auto it = range.first;it!=range.second;it++){
		if((long)it->second == (long)geomID){
			spaces.erase(it);
			break;
		}
	}
}
