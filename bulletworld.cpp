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
#include <iostream>

//#define  BULLETDEBUG
//#define BULLET_PARALLEL 1

//using namespace boost;
using namespace oxygen;
using namespace salt;

#ifdef BULLET_PARALLEL
 #define USE_PARALLEL_SOLVER 1 //experimental parallel solver
#define USE_PARALLEL_DISPATCHER 1

#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btSphereTriangleCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btSimulationIslandManager.h"

#ifdef USE_PARALLEL_DISPATCHER
#include "BulletMultiThreaded/SpuGatheringCollisionDispatcher.h"
#include "BulletMultiThreaded/PlatformDefinitions.h"

#ifdef USE_LIBSPE2
#include "BulletMultiThreaded/SpuLibspe2Support.h"
#elif defined (_WIN32)
#include "BulletMultiThreaded/Win32ThreadSupport.h"
#include "BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuGatheringCollisionTask.h"

#elif defined (USE_PTHREADS)

#include "BulletMultiThreaded/PosixThreadSupport.h"
#include "BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuGatheringCollisionTask.h"

#else
//other platforms run the parallel code sequentially (until pthread support or other parallel implementation is added)

#include "BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuGatheringCollisionTask.h"
#endif //USE_LIBSPE2

#ifdef USE_PARALLEL_SOLVER
#include "BulletMultiThreaded/btParallelConstraintSolver.h"
#include "BulletMultiThreaded/SequentialThreadSupport.h"


btThreadSupportInterface* createSolverThreadSupport(int maxNumThreads)
{
//#define SEQUENTIAL
#ifdef SEQUENTIAL
	SequentialThreadSupport::SequentialThreadConstructionInfo tci("solverThreads",SolverThreadFunc,SolverlsMemoryFunc);
	SequentialThreadSupport* threadSupport = new SequentialThreadSupport(tci);
	threadSupport->startSPU();
#else

#ifdef _WIN32
	Win32ThreadSupport::Win32ThreadConstructionInfo threadConstructionInfo("solverThreads",SolverThreadFunc,SolverlsMemoryFunc,maxNumThreads);
	Win32ThreadSupport* threadSupport = new Win32ThreadSupport(threadConstructionInfo);
	threadSupport->startSPU();
#elif defined (USE_PTHREADS)
	PosixThreadSupport::ThreadConstructionInfo solverConstructionInfo("solver", SolverThreadFunc,
																	  SolverlsMemoryFunc, maxNumThreads);
	
	PosixThreadSupport* threadSupport = new PosixThreadSupport(solverConstructionInfo);
	
#else
	SequentialThreadSupport::SequentialThreadConstructionInfo tci("solverThreads",SolverThreadFunc,SolverlsMemoryFunc);
	SequentialThreadSupport* threadSupport = new SequentialThreadSupport(tci);
	threadSupport->startSPU();
#endif
	
#endif

	return threadSupport;
}

#endif //USE_PARALLEL_SOLVER

#endif//USE_PARALLEL_DISPATCHER
#endif//BULLET_PARALLEL



#ifdef BULLETDEBUG
#include "GlutStuff.h"
#include "GLDebugDrawer.h"

#include "boost\thread.hpp"
/*#ifdef _WINDOWS
#include "Win32DemoApplication.h"
#define PlatformDemoApplication Win32DemoApplication
#else
*/
#include "GlutDemoApplication.h"
#define PlatformDemoApplication GlutDemoApplication
//#endif
#endif


#include "btBulletDynamicsCommon.h"
#include "btBulletCollisionCommon.h"

extern btDiscreteDynamicsWorld *lastWorld;

#ifdef BULLETDEBUG
GLDebugDrawer gDrawer;

//boost::mutex *worldMutex=nullptr;
//
//
//
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
		//while(!worldMutex)
			boost::this_thread::sleep(boost::posix_time::milliseconds(20));
		//////////////////////////////////////////////////////////////////////////
		//worldMutex->lock();
		if (m_dynamicsWorld)
		{
			//m_dynamicsWorld->stepSimulation(ms / 1000000.f);
			//optional but useful: debug drawing
			m_dynamicsWorld->debugDrawWorld();
		}

		renderme(); 
		//worldMutex->unlock();

		glFlush();

		swapBuffers();

	}

	virtual void displayCallback(){
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

		////////////////////////////////////////////////////////////////////////////
		////worldMutex not yet created, wait until it exists
		//while(!worldMutex){
		//	std::cerr << "mutex not created yet, waiting for it for 20ms"<< std::endl; 
		//boost::this_thread::sleep(boost::posix_time::milliseconds(200));
		//}
		////////////////////////////////////////////////////////////////////////////
		//worldMutex->lock();
		////std::cerr << "aquired WORLDLOCK for BulletDraw" << std::endl;

		renderme();

		//optional but useful: debug drawing to detect problems
		if (m_dynamicsWorld)
			m_dynamicsWorld->debugDrawWorld();
		//std::cerr << "unlocking WORLDLOCK for BulletDraw" << std::endl;
		//worldMutex->unlock();
		
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
		BulletTest* demo = new BulletTest(lastWorld);
		demo->myinit();
		demo->initPhysics();
		return demo;
	}

	void operator()(){
		
        std::string *argv = new std::string("test");
		char *argp= (char*)argv->c_str();
		gDrawer.setDebugMode(btIDebugDraw::DBG_DrawConstraints|btIDebugDraw::DBG_DrawConstraintLimits);
		boost::this_thread::sleep(boost::posix_time::milliseconds(15000));
		gDrawer.setDebugMode(btIDebugDraw::DBG_DrawWireframe+btIDebugDraw::DBG_DrawConstraints+btIDebugDraw::DBG_DrawConstraintLimits);
		lastWorld->setDebugDrawer(&gDrawer);
		glutmain(1,(char **)&argp,800,600,"Bullet Test",this);
	}
};

BulletTest::BulletTest(btDynamicsWorld *wrld){
		m_dynamicsWorld=(wrld);
		this->setCameraDistance(btScalar(50.0f));
		thrd = new boost::thread(boost::ref<BulletTest>(*this));
}

#endif

WorldImp::WorldImp() : PhysicsObjectImp(){
	
#ifdef BULLETDEBUG
	gDrawer.setDebugMode(btIDebugDraw::DBG_DrawConstraints|btIDebugDraw::DBG_DrawConstraintLimits);
#endif
}

void WorldImp::SetGravity(const Vector3f& gravity)
{
    btDiscreteDynamicsWorld *WorldImp =  worldID;
    WorldImp->setGravity( btVector3( gravity.x(),
                                     gravity.y(),
                                     gravity.z()
                                    )
                        );
}

salt::Vector3f WorldImp::GetGravity() const
{
    btDiscreteDynamicsWorld *WorldImp =  worldID;
    btVector3 dGravity;
    dGravity = WorldImp->getGravity();
    return Vector3f((float)dGravity.x(),
                    (float)dGravity.y(),
                    (float)dGravity.z()
                    );
}

void WorldImp::SetERP(float erp)
{
    btDiscreteDynamicsWorld *WorldImp = worldID;
    btContactSolverInfo solverInfo = WorldImp->getSolverInfo();
    solverInfo.m_erp = 0.9f;//erp;
}

float WorldImp::GetERP() const
{
    btDiscreteDynamicsWorld *WorldImp = worldID;
    btContactSolverInfo solverInfo = WorldImp->getSolverInfo();
    return solverInfo.m_erp;
}

void WorldImp::SetCFM(float cfm)
{
    btDiscreteDynamicsWorld *WorldImp =  worldID;
    btContactSolverInfo solverInfo = WorldImp->getSolverInfo();
    solverInfo.m_globalCfm = 0.01f;//cfm;
}

float WorldImp::GetCFM() const
{
    btDiscreteDynamicsWorld *WorldImp =  worldID;
    btContactSolverInfo solverInfo = WorldImp->getSolverInfo();
    return solverInfo.m_globalCfm;
}

void WorldImp::Step(float deltaTime)
{
	static int steps = 0;
	std::cout << "step " << steps++ <<std::endl;
	btDiscreteDynamicsWorld *WorldImp = worldID;
    WorldImp->stepSimulation(deltaTime,1,1.0/60);
}

bool WorldImp::GetAutoDisableFlag() const
{
    //TODO: Assess the importance of this function
    //Equivalent in Bullet not yet found
    return false;
}

void WorldImp::SetAutoDisableFlag(bool flag)
{
    //TODO: Assess the importance of this function
    //Equivalent in Bullet not yet found
}

void WorldImp::SetContactSurfaceLayer(float depth)
{
    //TODO: Should this use setMargin()? but it has no global default but rather
    //      per class defaults
    btDiscreteDynamicsWorld *WorldImp = worldID;
    WorldImp->getDispatchInfo().m_allowedCcdPenetration=btScalar(depth);
}

float WorldImp::GetContactSurfaceLayer() const
{
    //TODO: Should this use getMargin()? but it has no global default but rather
    //      per class defaults
    btDiscreteDynamicsWorld *WorldImp =  worldID;
    return (float) WorldImp->getDispatchInfo().m_allowedCcdPenetration;
}


long WorldImp::CreateWorld()
{
    btDiscreteDynamicsWorld *WorldImp;

#ifdef BULLET_PARALLEL
		const int maxProxies = 32766;
	const int maxOverlap = 65535;

	btBroadphaseInterface*	m_broadphase;

	btCollisionDispatcher*	m_dispatcher;

	class	btThreadSupportInterface*		m_threadSupportCollision;
	class	btThreadSupportInterface*		m_threadSupportSolver;

	btConstraintSolver*	m_solver;

	btCollisionAlgorithmCreateFunc*	m_boxBoxCF;

	btDefaultCollisionConfiguration* m_collisionConfiguration;


	#ifdef USE_PARALLEL_DISPATCHER
	m_threadSupportSolver = 0;
	m_threadSupportCollision = 0;
#endif

	m_dispatcher=0;
	btDefaultCollisionConstructionInfo cci;
	cci.m_defaultMaxPersistentManifoldPoolSize = 32768;
	m_collisionConfiguration = new btDefaultCollisionConfiguration(cci);
	
#ifdef USE_PARALLEL_DISPATCHER
	int maxNumOutstandingTasks = 4;

#ifdef USE_WIN32_THREADING

m_threadSupportCollision = new Win32ThreadSupport(Win32ThreadSupport::Win32ThreadConstructionInfo(
								"collision",
								processCollisionTask,
								createCollisionLocalStoreMemory,
								maxNumOutstandingTasks));
#else

#ifdef USE_LIBSPE2

   spe_program_handle_t * program_handle;
#ifndef USE_CESOF
                        program_handle = spe_image_open ("./spuCollision.elf");
                        if (program_handle == NULL)
                    {
                                perror( "SPU OPEN IMAGE ERROR\n");
                    }
                        else
                        {
                                printf( "IMAGE OPENED\n");
                        }
#else
                        extern spe_program_handle_t spu_program;
                        program_handle = &spu_program;
#endif
        SpuLibspe2Support* threadSupportCollision  = new SpuLibspe2Support( program_handle, maxNumOutstandingTasks);
#elif defined (USE_PTHREADS)
    PosixThreadSupport::ThreadConstructionInfo constructionInfo("collision",
								processCollisionTask,
								createCollisionLocalStoreMemory,
								maxNumOutstandingTasks);
    m_threadSupportCollision = new PosixThreadSupport(constructionInfo);
#else

	SequentialThreadSupport::SequentialThreadConstructionInfo colCI("collision",processCollisionTask,createCollisionLocalStoreMemory);
	SequentialThreadSupport* m_threadSupportCollision = new SequentialThreadSupport(colCI);
		
#endif //USE_LIBSPE2

///Playstation 3 SPU (SPURS)  version is available through PS3 Devnet
/// For Unix/Mac someone could implement a pthreads version of btThreadSupportInterface?
///you can hook it up to your custom task scheduler by deriving from btThreadSupportInterface
#endif


	m_dispatcher = new	SpuGatheringCollisionDispatcher(m_threadSupportCollision,maxNumOutstandingTasks,m_collisionConfiguration);
//	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);
#else
	
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);
#endif //USE_PARALLEL_DISPATCHER


	btVector3 worldAabbMin(-1000,-1000,-1000);
	btVector3 worldAabbMax(1000,1000,1000);

	

	m_broadphase = new btAxisSweep3(worldAabbMin,worldAabbMax,maxProxies);


	
#ifdef USE_PARALLEL_SOLVER
	m_threadSupportSolver = createSolverThreadSupport(maxNumOutstandingTasks);
	m_solver = new btParallelConstraintSolver(m_threadSupportSolver);
	//this solver requires the contacts to be in a contiguous pool, so avoid dynamic allocation
	m_dispatcher->setDispatcherFlags(btCollisionDispatcher::CD_DISABLE_CONTACTPOOL_DYNAMIC_ALLOCATION);
#else

	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();
	m_solver = solver;
	//default solverMode is SOLVER_RANDMIZE_ORDER. Warmstarting seems not to improve convergence, see 
	//solver->setSolverMode(0);//btSequentialImpulseConstraintSolver::SOLVER_USE_WARMSTARTING | btSequentialImpulseConstraintSolver::SOLVER_RANDMIZE_ORDER);
#endif //USE_PARALLEL_SOLVER




    //mBf = new btDbvtBroadphase();
    //mConfig = new btDefaultCollisionConfiguration;
    //mDispatcher = new btCollisionDispatcher(mConfig);
    //mSolver = new btSequentialImpulseConstraintSolver();
	//btParallelConstraintSolver *mSolver2 = new btParallelConstraintSolver(createSolverThreadSupport(4));
	WorldImp = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);//new btDiscreteDynamicsWorld(mDispatcher,mBf,(btConstraintSolver *)mSolver,mConfig);
    WorldImp->setGravity(btVector3(0.0f,0.0f,-9.81f));
    //WorldImp->getSimulationIslandManager()->setSplitIslands(false);


#else
    btBroadphaseInterface *mBf;
    btDefaultCollisionConfiguration *mConfig;
    btCollisionDispatcher *mDispatcher;
    btSequentialImpulseConstraintSolver *mSolver;

    mBf = new btDbvtBroadphase();
    mConfig = new btDefaultCollisionConfiguration;
    mDispatcher = new btCollisionDispatcher(mConfig);
    mSolver = new btSequentialImpulseConstraintSolver();
	
    //:TODO: remove debug code
	WorldImp = new btDiscreteDynamicsWorld(mDispatcher,mBf,(btConstraintSolver *)mSolver,mConfig);
    WorldImp->setGravity(btVector3(0.0f,-9.81f,0.0f));
	WorldImp->getDispatchInfo().m_allowedCcdPenetration=btScalar(0.001);
	//worldMutex = new boost::mutex();
#endif
#ifdef BULLETDEBUG    
	BulletTest *test = new BulletTest(WorldImp);
#endif
	lastWorld = WorldImp;
	worldID = WorldImp;
    return (long) WorldImp;
}

void WorldImp::DestroyWorld()
{
    btDiscreteDynamicsWorld *m_dynamicsWorld = worldID;
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

