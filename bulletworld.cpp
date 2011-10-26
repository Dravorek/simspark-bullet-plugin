/* -*- mode: c++ -*-

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

#include "bulletworld.h"



//using namespace boost;
using namespace oxygen;
using namespace salt;

WorldImp::WorldImp() : PhysicsObjectImp(){
}

void WorldImp::SetGravity(const Vector3f& gravity, long worldID)
{
    btDynamicsWorld *WorldImp = (btDynamicsWorld *) worldID;
    WorldImp->setGravity( btVector3( gravity.x(),
                                     gravity.y(),
                                     gravity.z()
                                    )
                        );
}

salt::Vector3f WorldImp::GetGravity(long worldID) const
{
    btDynamicsWorld *WorldImp = (btDynamicsWorld *) worldID;
    btVector3 dGravity;
    dGravity = WorldImp->getGravity();
    return Vector3f((float)dGravity.x(),
                    (float)dGravity.y(),
                    (float)dGravity.z()
                    );
}

void WorldImp::SetERP(float erp, long worldID)
{
    btDynamicsWorld *WorldImp = (btDynamicsWorld *) worldID;
    btContactSolverInfo solverInfo = WorldImp->getSolverInfo();
    solverInfo.m_erp = erp;
}

float WorldImp::GetERP(long worldID) const
{
    btDynamicsWorld *WorldImp = (btDynamicsWorld *) worldID;
    btContactSolverInfo solverInfo = WorldImp->getSolverInfo();
    return solverInfo.m_erp;
}

void WorldImp::SetCFM(float cfm, long worldID)
{
    btDynamicsWorld *WorldImp = (btDynamicsWorld *) worldID;
    btContactSolverInfo solverInfo = WorldImp->getSolverInfo();
    solverInfo.m_globalCfm = cfm;
}

float WorldImp::GetCFM(long worldID) const
{
    btDynamicsWorld *WorldImp = (btDynamicsWorld *) worldID;
    btContactSolverInfo solverInfo = WorldImp->getSolverInfo();
    return solverInfo.m_globalCfm;
}

void WorldImp::Step(float deltaTime, long worldID)
{
    btDynamicsWorld *WorldImp = (btDynamicsWorld *) worldID;
    WorldImp->stepSimulation(btScalar(deltaTime));
}

bool WorldImp::GetAutoDisableFlag(long worldID) const
{
    //TODO: Assess the importance of this function
    //Equivalent in Bullet not yet found
    return false;
}

void WorldImp::SetAutoDisableFlag(bool flag, long worldID)
{
    //TODO: Assess the importance of this function
    //Equivalent in Bullet not yet found
}

void WorldImp::SetContactSurfaceLayer(float depth, long worldID)
{
    //TODO: Should this use setMargin()? but it has no global default but rather
    //      per class defaults
    btDynamicsWorld *WorldImp = (btDynamicsWorld *) worldID;
    WorldImp->getDispatchInfo().m_allowedCcdPenetration=btScalar(depth);
}

float WorldImp::GetContactSurfaceLayer(long worldID) const
{
    //TODO: Should this use getMargin()? but it has no global default but rather
    //      per class defaults
    btDynamicsWorld *WorldImp = (btDynamicsWorld *) worldID;
    return (float) WorldImp->getDispatchInfo().m_allowedCcdPenetration;
}

long WorldImp::CreateWorld()
{
    btBroadphaseInterface *mBf;
    btDefaultCollisionConfiguration *mConfig;
    btCollisionDispatcher *mDispatcher;
    btSequentialImpulseConstraintSolver *mSolver;
    btDynamicsWorld *WorldImp;

    mBf = new btDbvtBroadphase();
    mConfig = new btDefaultCollisionConfiguration;
    mDispatcher = new btCollisionDispatcher(mConfig);
    mSolver = new btSequentialImpulseConstraintSolver();
    WorldImp = new btDiscreteDynamicsWorld(mDispatcher,mBf,(btConstraintSolver *)mSolver,mConfig);
    WorldImp->setGravity(btVector3(0.0f,-9.81f,0.0f));
    
    return (long) WorldImp;
}

void WorldImp::DestroyWorld(long worldID)
{
    btDynamicsWorld *m_dynamicsWorld = (btDynamicsWorld *)worldID;
    for(int i = m_dynamicsWorld->getNumConstraints()-1;i>=0 ;i--){
        btTypedConstraint *obj = m_dynamicsWorld->getConstraint(i);
        m_dynamicsWorld->removeConstraint(obj);
        delete obj;
    }

    for(int i = m_dynamicsWorld->getNumCollisionObjects()-1;i>=0 ;i--){
        btCollisionObject *obj = m_dynamicsWorld->getCollisionObjectArray()[i];
        btRigidBody *bdy = btRigidBody::upcast(obj);
        if(bdy && bdy->getMotionState())
        {
            delete bdy->getMotionState();
        }
        m_dynamicsWorld->removeCollisionObject(obj);
        delete obj;
    }

    btBroadphaseInterface *bf = m_dynamicsWorld->getBroadphase();
    btConstraintSolver *slv = m_dynamicsWorld->getConstraintSolver();
    btCollisionDispatcher *disp = (btCollisionDispatcher *)m_dynamicsWorld->getDispatcher();
    btCollisionConfiguration *cfg = disp->getCollisionConfiguration();

    delete m_dynamicsWorld;
    delete bf;
    delete slv;
    delete disp;
    delete cfg;
}
