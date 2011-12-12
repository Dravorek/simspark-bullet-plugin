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

#include "GlutStuff.h"
#include "btBulletDynamicsCommon.h"

#include "GLDebugDrawer.h"

#include "boost\thread.hpp"

#include "ZGVdefines.h"

/*#ifdef _WINDOWS
#include "Win32DemoApplication.h"
#define PlatformDemoApplication Win32DemoApplication
#else
*/#include "GlutDemoApplication.h"
#define PlatformDemoApplication GlutDemoApplication
//#endif

GLDebugDrawer gDrawer;

boost::mutex *worldMutex=nullptr;

struct TreeVizCallable{
	zeitgeist::Core *ref;
	TreeVizCallable(zeitgeist::Core * ptr) : ref(ptr){
		auto thrd = new boost::thread(&TreeVizCallable::start,this);
	}
	void start(){
		std::cerr << "starting tree visualizer thread" << std::endl;
		ZeitgeistUpdater::init(ref);
		std::cerr << "ended visualizer thread" << std::endl;
	}
};

class BulletTest :
    public PlatformDemoApplication
{
    btBroadphaseInterface *mBf;
    btDefaultCollisionConfiguration *mConfig;
    btCollisionDispatcher *mDispatcher;
    btSequentialImpulseConstraintSolver *mSolver;
    btAlignedObjectArray<btCollisionShape *> mCollisionShapes;

public:
	boost::thread *thrd;
    BulletTest(btDynamicsWorld *wrld);
    virtual ~BulletTest()
	{
		if(thrd)
			delete thrd;
	}

	void	initPhysics(){}

	void	exitPhysics(){}

	virtual void clientMoveAndDisplay(){
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

		//simple dynamics world doesn't handle fixed-time-stepping
		float ms = getDeltaTimeMicroseconds();

		gDrawer.setDebugMode(btIDebugDraw::DBG_DrawWireframe+btIDebugDraw::DBG_DrawConstraints+btIDebugDraw::DBG_DrawConstraintLimits);

		///step the simulation
		
		//////////////////////////////////////////////////////////////////////////
		//worldMutex not yet created, wait until it exists
		while(!worldMutex)
			boost::this_thread::sleep(boost::posix_time::milliseconds(20));
		//////////////////////////////////////////////////////////////////////////
		worldMutex->lock();
		if (m_dynamicsWorld)
		{
			//m_dynamicsWorld->stepSimulation(ms / 1000000.f);
			//optional but useful: debug drawing
			m_dynamicsWorld->debugDrawWorld();
		}

		renderme(); 
		worldMutex->unlock();

		glFlush();

		swapBuffers();

	}

	virtual void displayCallback(){
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

		//////////////////////////////////////////////////////////////////////////
		//worldMutex not yet created, wait until it exists
		while(!worldMutex){
			std::cerr << "mutex not created yet, waiting for it for 20ms"<< std::endl; 
			boost::this_thread::sleep(boost::posix_time::milliseconds(20));
		}
		//////////////////////////////////////////////////////////////////////////
		worldMutex->lock();
		//std::cerr << "aquired WORLDLOCK for BulletDraw" << std::endl;

		renderme();

		//optional but useful: debug drawing to detect problems
		if (m_dynamicsWorld)
			m_dynamicsWorld->debugDrawWorld();
		//std::cerr << "unlocking WORLDLOCK for BulletDraw" << std::endl;
		worldMutex->unlock();
		
		glFlush();
		swapBuffers();
	}
	virtual void	clientResetScene(){
		exitPhysics();
		initPhysics();
		//this->getDynamicsWorld()->setDebugDrawer(&gDrawer);
	}

	static DemoApplication* Create()
	{
		BulletTest* demo = new BulletTest(nullptr);
		demo->myinit();
		demo->initPhysics();
		return demo;
	}

	void operator()(){
		
        std::string *argv = new std::string("test");
		char *argp= (char*)argv->c_str();
		glutmain(1,(char **)&argp,800,600,"Bullet Test",this);
	}
};

BulletTest::BulletTest(btDynamicsWorld *wrld){
		m_dynamicsWorld=(wrld);
		this->setCameraDistance(btScalar(50.0f));
		thrd = new boost::thread(boost::ref<BulletTest>(*this));
}

WorldImp::WorldImp() : PhysicsObjectImp(){
	gDrawer.setDebugMode(btIDebugDraw::DBG_DrawConstraints+btIDebugDraw::DBG_DrawConstraintLimits);
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
	worldMutex = new boost::mutex();
    
	BulletTest *test = new BulletTest(WorldImp);
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

boost::mutex *WorldImp::GetMutex(zeitgeist::Core *ptr){
	auto type = new TreeVizCallable(ptr);
	return worldMutex;
}