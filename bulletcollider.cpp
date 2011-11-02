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
#include <oxygen/physicsserver/collider.h>
#include <oxygen/physicsserver/world.h>
#include <oxygen/sceneserver/scene.h>

using namespace oxygen;
using namespace salt;
using namespace boost;
using namespace std;

ColliderImp::ColliderImp() : PhysicsObjectImp()
{
}

ColliderImp::~ColliderImp()
{
}

Collider* ColliderImp::GetColliderPointer(long geomID){
    btCollisionObject *BulletGeom = (btCollisionObject *) geomID;
    //TODO: not very safe, check if geomID reallly is the right type
    return static_cast<Collider*>(BulletGeom->getUserPointer());
}

void ColliderImp::SetRotation(const Matrix& rot, long geomID)
{
    btCollisionObject *BulletGeom = (btCollisionObject *) geomID;
    btMatrix3x3 BulletMatrix;
    GenericPhysicsMatrix& matrixRef = (GenericPhysicsMatrix&) BulletMatrix; 
    ConvertRotationMatrix(rot, matrixRef);
    btTransform trans = BulletGeom->getWorldTransform();
    trans.setBasis(BulletMatrix);
    BulletGeom->setWorldTransform(trans);
}

void ColliderImp::SetPosition(const Vector3f& globalPos, long geomID)
{
    btCollisionObject *BulletGeom = (btCollisionObject *) geomID;
    btTransform trans = BulletGeom->getWorldTransform();
    trans.setOrigin(btVector3(
            btScalar(globalPos[0]),
            btScalar(globalPos[1]),
            btScalar(globalPos[2])));    
    BulletGeom->setWorldTransform(trans);
}

void ColliderImp::SetLocalPosition(const Vector3f& pos, long geomID)
{
    SetPosition(pos,geomID);
}

Vector3f ColliderImp::GetPosition(long geomID) const
{
    btCollisionObject  *BulletGeom = (btCollisionObject  *) geomID;
    btTransform trans = BulletGeom->getWorldTransform();
    return Vector3f(
                (float)trans.getOrigin().x(),
                (float)trans.getOrigin().y(),
                (float)trans.getOrigin().z()
            );
}

long ColliderImp::GetParentSpaceID(long geomID)
{
    //TODO: check how safe returning 0 is
    return 0l;
}

bool ColliderImp::Intersect(boost::shared_ptr<Collider> collider, long geomID)
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
    return false;
}

void ColliderImp::DestroyGeom(long geomID)
{
    btCollisionObject *BulletGeom = (btCollisionObject *) geomID;
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
    delete BulletGeom;

}

///<todo>see this damnit</todo>
///<!todo>work</todo>
///!<todo>now</todo>
///<TODO>now</TODO> do something
//<TODO>now</TODO> do something

void ColliderImp::TransformSetGeom(long parentGeomID, long geomID){
    /// TODO : figure out how to handle parenting in bullet
    /*dGeomID parentODEGeom = (dGeomID) parentGeomID;
    dGeomID ODEGeom = (dGeomID) geomID;
    dGeomTransformSetGeom(parentODEGeom, ODEGeom);*/
}

void ColliderImp::SetSpace(long spaceID, long geomID, Collider* collider){
    //dSpaceID ODESpace = (dSpaceID) spaceID;
    // TODO add space handling if necessary
    
    btCollisionObject *BulletGeom = (btCollisionObject *) geomID;
    
    if(BulletGeom->getUserPointer() ==0){
        BulletGeom->setUserPointer(collider);
    }


}

void ColliderImp::SetBody(long bodyID, long geomID){
    /// TODO : figure out how to handle parenting in bullet
    /*dBodyID ODEBody = (dBodyID) bodyID;
    dGeomID ODEGeom = (dGeomID) geomID;
    dGeomSetBody(ODEGeom, ODEBody);*/
}

void ColliderImp::RemoveFromSpace(long geomID, long spaceID){
    // TODO add space handling if necessary
    /*dGeomID ODEGeom = (dGeomID) geomID;
    dSpaceID ODESpace = (dSpaceID) spaceID;
    dSpaceRemove(ODESpace, ODEGeom);*/
}
