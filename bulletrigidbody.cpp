/* -*- mode: c++; c-basic-offset: 4; indent-tabs-mode: nil -*-

   this file is part of rcssserver3D
   Fri May 9 2003
   Copyright (C) 2002,2003 Koblenz University
   Copyright (C) 2003 RoboCup Soccer Server 3D Maintenance Group
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

#include "bulletrigidbody.h"
#include "bulletworld.h"

//TODO get default mass from world or sonething
#define DEFAULT_MASS 0.0f


using namespace oxygen;
//using namespace boost;
using namespace salt;
using namespace std;

typedef btGeom* dBodyID;

RigidBodyImp::RigidBodyImp() : BodyImp(){

}

RigidBody* RigidBodyImp::GetBodyPointer()
{
    dBodyID BulletBody = (dBodyID) btID;
    if(!BulletBody->isRigidBody){
		std::cerr << "WARNING (BulletRigidBodyImp): tried to treat a CollisionObject as a RigidBody" <<std::endl;
		//return 0;
	}
	//return static_cast<RigidBody*>(((btRigidBody *)BulletBody->obj)->getUserPointer());
	return reinterpret_cast<RigidBody *>(userPointer);
}

void RigidBodyImp::Enable()
{
    dBodyID BulletBody = (dBodyID) btID;
    if(!BulletBody->isRigidBody){
		std::cerr << "WARNING (BulletRigidBodyImp): tried to treat a CollisionObject as a RigidBody" <<std::endl;
		return;
	}
	btRigidBody *bdy = ((btRigidBody *)BulletBody->obj);
    bdy->activate(true);
}

void RigidBodyImp::Disable()
{
    dBodyID BulletBody = (dBodyID) btID;
    if(!BulletBody->isRigidBody){
		std::cerr << "WARNING (BulletRigidBodyImp): tried to treat a CollisionObject as a RigidBody" <<std::endl;
		return;
	}
	btRigidBody *bdy = ((btRigidBody *)BulletBody->obj);
    //TODO: maybe WANTS_DEACTIVATION ismoreappropriate
    bdy->setActivationState(DISABLE_SIMULATION);
}

bool RigidBodyImp::IsEnabled() const
{
    dBodyID BulletBody = (dBodyID) btID;
    if(!BulletBody->isRigidBody){
		std::cerr << "WARNING (BulletRigidBodyImp): tried to treat a CollisionObject as a RigidBody" <<std::endl;
		return true;
	}
	return (((btRigidBody *)BulletBody->obj)->isActive());
}

void RigidBodyImp::UseGravity(bool f)
{
    //TODO decide if STATIC or Kinematic is appropriate
    //Also if the gravity from the world should be read
    dBodyID BulletBody = (dBodyID) btID;
	if(!BulletBody->isRigidBody){
		std::cerr << "WARNING (BulletRigidBodyImp): tried to treat a CollisionObject as a RigidBody" <<std::endl;
		return;
	}
	if (f == true)
        {
            //erase the CF_KINEMATIC_OBJECT and CF_STATIC_FLAGS
            ((btRigidBody *)BulletBody->obj)->setFlags(((btRigidBody *)BulletBody->obj)->getFlags() 
                & ~(((btRigidBody *)BulletBody->obj)->CF_KINEMATIC_OBJECT 
                  | ((btRigidBody *)BulletBody->obj)->CF_STATIC_OBJECT));
            //// body is affected by gravity
            //((btRigidBody *)BulletBody->obj)->setGravity(btVector3( 
            //            btScalar(0.0f),
            //            btScalar(-9.81f),
            //            btScalar(0.0f)
            //            ));
        }
    else
        {
            // body is not affected by gravity
            // set the CF_KINEMATIC_OBJECT flag
            ((btRigidBody *)BulletBody->obj)->setFlags(((btRigidBody *)BulletBody->obj)->getFlags() |
                                      ((btRigidBody *)BulletBody->obj)->CF_KINEMATIC_OBJECT);
            //((btRigidBody *)BulletBody->obj)->setGravity(btVector3( 
            //            btScalar(0.0f),
            //            btScalar(0.0f),
            //            btScalar(0.0f)
            //            ));
        }
}

bool RigidBodyImp::UsesGravity() const
{
    dBodyID BulletBody = (dBodyID) btID;
    if(!BulletBody->isRigidBody){
		std::cerr << "WARNING (BulletRigidBodyImp): tried to treat a CollisionObject as a RigidBody" <<std::endl;
		return false;
	}
	//if the rigid body has the STATIC or KINEMATIC flag
    //it's not affected by gravity
    return ((
              ((btRigidBody *)BulletBody->obj)->CF_STATIC_OBJECT 
              | ((btRigidBody *)BulletBody->obj)->CF_KINEMATIC_OBJECT
            )
            & ((btRigidBody *)BulletBody->obj)->getFlags()
           );
    //return !(((btRigidBody *)BulletBody->obj)->getGravity().isZero());
}

long RigidBodyImp::CreateBody(WorldInt *worldID)
{
    // create the managed body
   dBodyID NewBdy = new btGeom();

   NewBdy->shp = new btEmptyShape();
   //HACK: shapes that are created in the RigidBody Class don't have their own
   //Colliders and to identify them they point the userPointer to themselves
   NewBdy->shp->setUserPointer(NewBdy->shp);
   //NewBdy->shp->calculateLocalInertia( btScalar(DEFAULT_MASS), btVector3(
   //                                                     btScalar(0.0f),
   //                                                     btScalar(0.0f),
   //                                                     btScalar(0.0f)
   //                                                 ));
   btMotionState *MState = new btDefaultMotionState();
   NewBdy->obj = new btRigidBody( btScalar(DEFAULT_MASS * 1.0f),
                                  MState,
                                  NewBdy->shp
                                  );
   	NewBdy->obj->setUserPointer(static_cast<BodyInt * >(this));
   btDiscreteDynamicsWorld *Wrld = static_cast<WorldImp *>(worldID)->worldID;
   //NewBdy->obj->setCollisionFlags(btCollisionObject::CF_KINEMATIC_OBJECT | NewBdy->obj->getCollisionFlags());
   Wrld->addRigidBody(((btRigidBody *)NewBdy->obj));
   NewBdy->obj->setFriction(100.0);
   static_cast<btRigidBody *>(NewBdy->obj)->activate(true);
   static_cast<btRigidBody *>(NewBdy->obj)->setActivationState(DISABLE_DEACTIVATION);
   NewBdy->wrld = Wrld;
   btID=NewBdy;
   return (long) NewBdy; 
}

void RigidBodyImp::DestroyRigidBody()
{
    //TODO check where the shape gets deleted and delete btGeom there
    dBodyID BulletBody = (dBodyID) btID;
	if(!BulletBody->isRigidBody){
		std::cerr << "WARNING (BulletRigidBodyImp): tried to delete a CollisionObject as RigidBody" <<std::endl;
		return;
	}
	if(((btRigidBody *)BulletBody->obj)->getNumConstraintRefs()){
		//:TODO: RigidBody is connected to a joint, can't remove
		// unless joint is destroyed but reference to jointwrapper can't
		// be obtained here, either another map is needed or changing the API
	}
	else
	{
		btMotionState *MState = ((btRigidBody *)BulletBody->obj)->getMotionState();
		delete ((btRigidBody *)BulletBody->obj);
		BulletBody->obj = 0;
		delete MState;
		//No shape has been set, delete placeholder
		//if(BulletBody->shp->getShapeType()== EMPTY_SHAPE_PROXYTYPE )
		if( !CheckShapeHasCollider() )
		{
			delete BulletBody->shp;
			delete BulletBody;
		}
	}
    
}

void RigidBodyImp::BodySetData(RigidBody* rb)
{
    dBodyID BulletBody = (dBodyID) btID;
    if(!BulletBody->isRigidBody){
		std::cerr << "WARNING (BulletRigidBodyImp): tried to set RigidBody userdata on a CollisionObject" <<std::endl;
		return;
	}
	userPointer = rb;
	((btRigidBody *)BulletBody->obj)->setUserPointer(static_cast<BodyInt *>(this));
}

void RigidBodyImp::SetMass(float mass)
{
    //TODO remove rigid body from world and read it after adjusting mass
    std::cout << "setting body mass"<< std::endl;
	dBodyID BulletBody = (dBodyID) btID;
    if(!BulletBody->isRigidBody){
		std::cerr << "WARNING (BulletRigidBodyImp): tried to set mass on a CollisionObject" <<std::endl;
		return;
	}
	btVector3 inertia = ((btRigidBody *)BulletBody->obj)->getInvInertiaDiagLocal();
    //HACK possibly reduces precision
    //btVector3 inertia2;
	//inertia2.setValue(inertia.x() != btScalar(0.0) ? btScalar(1.0) / inertia.x(): btScalar(0.0),
    //                 inertia.y() != btScalar(0.0) ? btScalar(1.0) / inertia.y(): btScalar(0.0),
    //                 inertia.z() != btScalar(0.0) ? btScalar(1.0) / inertia.z(): btScalar(0.0));
    //((btRigidBody *)BulletBody->obj)->setMassProps( btScalar(mass),inertia);
    BulletBody->wrld->removeRigidBody(((btRigidBody *)BulletBody->obj));
    if(BulletBody->shp->getShapeType()!=EMPTY_SHAPE_PROXYTYPE )
	BulletBody->shp->calculateLocalInertia(btScalar(mass),inertia);
	((btRigidBody *)BulletBody->obj)->setMassProps( btScalar(mass),inertia);
	((btRigidBody *)BulletBody->obj)->updateInertiaTensor();

	//((btRigidBody *)BulletBody->obj)->setInvInertiaDiagLocal(inertia);

	BulletBody->wrld->addRigidBody(((btRigidBody *)BulletBody->obj));
	static_cast<btRigidBody *>(BulletBody->obj)->activate(true);
	((btRigidBody *)BulletBody->obj)->setActivationState(DISABLE_DEACTIVATION);
}

float RigidBodyImp::GetMass() const
{
    dBodyID BulletBody = (dBodyID) btID;
    if(!BulletBody->isRigidBody){
		std::cerr << "WARNING (BulletRigidBodyImp): tried to get mass from a CollisionObject" <<std::endl;
		return 0.0f;
	}
    return ((btRigidBody *)BulletBody->obj)->getInvMass()==0
                            ?0.0f
                            :1.0f/((btRigidBody *)BulletBody->obj)->getInvMass();
}

Vector3f RigidBodyImp::AddMass(const btMass& Mass, const Matrix& matrix, Vector3f massTrans)
{
	std::cerr << "(RigidBodyImp) ERROR called unimplemented method addMass" << std::endl;
	std::cerr << "mat:"; matrix.Dump();
	std::cerr << "vec"; massTrans.Dump();


    dBodyID BulletBody = (dBodyID) btID;
	btVector3 vec = ((btRigidBody *)BulletBody->obj)->getInvInertiaDiagLocal();
	vec.setValue(vec.x()!=0.0?1.0/vec.x():0.0f,
				 vec.y()!=0.0?1.0/vec.y():0.0f,
				 vec.z()!=0.0?1.0/vec.z():0.0f);
	((btRigidBody *)BulletBody->obj)->setMassProps(1.0/((btRigidBody *)BulletBody->obj)->getInvMass(),vec);
	//dMass transMass(ODEMass);

    //dMatrix3 ODEMatrix;
    //GenericPhysicsMatrix& matrixRef = (GenericPhysicsMatrix&) ODEMatrix;
    //ConvertRotationMatrix(matrix, matrixRef);
    //dMassRotate(&transMass, ODEMatrix);

    //const Vector3f& trans(matrix.Pos());
    //dMassTranslate(&transMass,trans[0],trans[1],trans[2]);

    //dMassTranslate(&transMass,massTrans[0],massTrans[1],massTrans[2]);
    //
    //dMass bodyMass;
    //dBodyGetMass(BulletBody, &bodyMass);
    //dMassAdd(&bodyMass, &transMass);

    ///** ODE currently requires that the center mass is always in the
    //    origin of the body
    //*/
    //Vector3f trans2(bodyMass.c[0], bodyMass.c[1], bodyMass.c[2]);

    //dMassTranslate(&bodyMass, -trans2[0], -trans2[1], -trans2[2]);
    //bodyMass.c[0] = bodyMass.c[1] = bodyMass.c[2] = 0.0f;
    //dBodySetMass(BulletBody, (const btMass*)&bodyMass);

    //// Move body so mass is at right position again
    //SetPosition(GetPosition(bodyID) + trans2, bodyID);
    //
    //// Keep track of total translation of mass
    //return massTrans - trans2;
    //
    return Vector3f();
}

bool RigidBodyImp::CheckCompoundStatus(bool prepareAddition)
{
    dBodyID BulletBody = (dBodyID) btID;
    bool isCompound = (BulletBody->shp->getShapeType() == COMPOUND_SHAPE_PROXYTYPE);
    //TODO: evaluate if this is faster:
    /*
        return isCompound;
    */
    if(isCompound)
    {
        if(prepareAddition)
        {
            return true;//isCompound
        }
        else
        {
            btCompoundShape *shp = static_cast<btCompoundShape *>(BulletBody->shp);
            //is the compound shape irreducable
            if(shp->getNumChildShapes()>1 || 
                (shp->getNumChildShapes()==1 && shp->getChildTransform(0).getOrigin()!=btTransform::getIdentity().getOrigin() )
                || shp->getUserPointer()!= shp)
            {
                return true;//isCompound
            }
            else //delete Compound
            {
                btCollisionShape *shp_new;
                if(shp->getNumChildShapes()==1)
                {
                    shp_new = shp->getChildShape(0);
                }
                else
                {
                    shp_new = new btEmptyShape();
                    shp_new->setUserPointer(shp_new);
                }
                //TODO: remove and re-add rigidbody to World
                ((btRigidBody *)BulletBody->obj)->setCollisionShape(shp_new);
                delete shp;
                BulletBody->shp = shp_new;
                return false;
            }
        }
    }
    else //is not a compound
    {
        if(prepareAddition){
            btCompoundShape *shp_new = new btCompoundShape();
            shp_new->setUserPointer(shp_new);
            if(CheckShapeHasCollider())
            {
                shp_new->addChildShape(btTransform::getIdentity(),BulletBody->shp);
            }
            else
            {
                delete BulletBody->shp;
                BulletBody->shp=0;
            }
            BulletBody->shp = shp_new;
            return true;
        }
        else
        {
            return false;//isCompound
        }
    }
}
    
bool RigidBodyImp::CheckShapeHasCollider()
{
    //HACK: as described in createRigidBody()
    dBodyID BulletBody = (dBodyID) btID;
    return BulletBody->shp->getUserPointer()!=BulletBody->shp;
}
    
void RigidBodyImp::SetMassParameters(const float mass,
                                     const salt::Vector3f& center_of_mass
                                     )
{
    //TODO: apply btMass->transform to the rigid body
    dBodyID BulletBody = (dBodyID) btID;
    btMass& BulletMass = (btMass&) mass;

    this->SetMass(BulletMass.mass);
    //dBodySetMass(BulletBody, &ODEMass);
}

void RigidBodyImp::PrepareSphere(btMass& mass, float density, float radius) const
{
    //TODO: calculate weight of the sphere
    //btMassSetSphere(&mass, density, radius);
    mass.transform = btTransform::getIdentity();
    mass.mass = btScalar( density * (radius * radius * radius * M_PI * 4.0/3.0) );
}

void RigidBodyImp::SetSphere(float density, float radius)
{
    dBodyID BulletBody = (dBodyID) btID;
    btMass BulletMass;
    PrepareSphere(BulletMass, density, radius);
    SetMassParameters(BulletMass.mass,Vector3f(BulletMass.transform.getOrigin().m_floats));
}

salt::Vector3f RigidBodyImp::AddSphere(float density, float radius, const Matrix& matrix, salt::Vector3f massTrans)
{
    btMass BulletMass;
    PrepareSphere(BulletMass, density, radius);
    return AddMass(BulletMass, matrix, massTrans);
}

void RigidBodyImp::PrepareSphereTotal(btMass& mass, float total_mass, float radius) const
{
    mass.mass = btScalar(total_mass);
    mass.transform = btTransform::getIdentity();
}

void RigidBodyImp::SetSphereTotal(float total_mass, float radius)
{
    dBodyID BulletBody = (dBodyID) btID;
    btMass BulletMass;
    PrepareSphereTotal(BulletMass, total_mass, radius);
    SetMassParameters(BulletMass.mass,Vector3f(BulletMass.transform.getOrigin().m_floats));
}

salt::Vector3f RigidBodyImp::AddSphereTotal(float total_mass, float radius, const Matrix& matrix, salt::Vector3f massTrans)
{
    btMass BulletMass;
    PrepareSphereTotal(BulletMass, total_mass, radius);
    return AddMass(BulletMass, matrix, massTrans);
}

void RigidBodyImp::PrepareBox(btMass& mass, float density, const Vector3f& size) const
{
    mass.transform = btTransform::getIdentity();
    mass.mass = btScalar( size[0] * size[1] * size[2] * density );
}

void RigidBodyImp::SetBox(float density, const Vector3f& size)
{
    dBodyID BulletBody = (dBodyID) btID;
    btMass BulletMass;
    PrepareBox(BulletMass, density, size);
    SetMassParameters(BulletMass.mass,Vector3f(BulletMass.transform.getOrigin().m_floats));
}

salt::Vector3f RigidBodyImp::AddBox(float density, const Vector3f& size, const Matrix& matrix, salt::Vector3f massTrans)
{
    btMass BulletMass;
    PrepareBox(BulletMass, density, size);
    return AddMass(BulletMass, matrix, massTrans);
}

void RigidBodyImp::PrepareBoxTotal(btMass& mass, float total_mass, const Vector3f& size) const
{
    mass.mass = btScalar( total_mass );
    mass.transform = btTransform::getIdentity();
}

void RigidBodyImp::SetBoxTotal(float total_mass, const Vector3f& size)
{
    dBodyID BulletBody = (dBodyID) btID;
    btMass BulletMass;
    PrepareBoxTotal(BulletMass, total_mass, size);
    SetMassParameters(BulletMass.mass,Vector3f(BulletMass.transform.getOrigin().m_floats));
}

salt::Vector3f RigidBodyImp::AddBoxTotal(float total_mass, const Vector3f& size, const Matrix& matrix, salt::Vector3f massTrans)
{
    btMass BulletMass;
    PrepareBoxTotal(BulletMass, total_mass, size);
    return AddMass(BulletMass, matrix, massTrans);
}

void RigidBodyImp::PrepareCylinder (btMass& mass, float density, float radius, float length) const
{
    //// direction: (1=x, 2=y, 3=z)
    //int direction = 3;
    mass.mass = btScalar( density * M_PI * radius * radius * length );
    mass.transform = btTransform::getIdentity();
}

void RigidBodyImp::SetCylinder (float density, float radius, float length)
{
    dBodyID BulletBody = (dBodyID) btID;
    btMass BulletMass;
    PrepareCylinder(BulletMass, density, radius, length);
    SetMassParameters(BulletMass.mass,Vector3f(BulletMass.transform.getOrigin().m_floats));
}

salt::Vector3f RigidBodyImp::AddCylinder(float density, float radius, float length, const Matrix& matrix, salt::Vector3f massTrans)
{
    btMass BulletMass;
    PrepareCylinder(BulletMass, density, radius, length);
    return AddMass(BulletMass, matrix, massTrans);
}

void RigidBodyImp::PrepareCylinderTotal(btMass& mass, float total_mass, float radius, float length) const
{
    //// direction: (1=x, 2=y, 3=z)
    //int direction = 3;
    mass.mass = btScalar( total_mass );
    mass.transform = btTransform::getIdentity();
}

void RigidBodyImp::SetCylinderTotal(float total_mass, float radius, float length)
{
    dBodyID BulletBody = (dBodyID) btID;
    btMass BulletMass;
    PrepareCylinderTotal(BulletMass, total_mass, radius, length);
    SetMassParameters(BulletMass.mass,Vector3f(BulletMass.transform.getOrigin().m_floats));
}

salt::Vector3f RigidBodyImp::AddCylinderTotal(float total_mass, float radius, float length, const Matrix& matrix, salt::Vector3f massTrans)
{
    btMass BulletMass;
    PrepareCylinderTotal(BulletMass, total_mass, radius, length);
    return AddMass(BulletMass, matrix, massTrans);
}

void RigidBodyImp::PrepareCapsule (btMass& mass, float density, float radius, float length) const
{
    //// direction: (1=x, 2=y, 3=z)
    //int direction = 3;
    mass.mass = btScalar( M_PI * radius * radius * ((4.0/3.0)*radius + length) );
    mass.transform = btTransform::getIdentity();
}

void RigidBodyImp::SetCapsule (float density, float radius, float length)
{
    dBodyID BulletBody = (dBodyID) btID;
    btMass BulletMass;
    PrepareCapsule(BulletMass, density, radius, length);
    SetMassParameters(BulletMass.mass,Vector3f(BulletMass.transform.getOrigin().m_floats));
}

salt::Vector3f RigidBodyImp::AddCapsule (float density, float radius, float length, const Matrix& matrix, salt::Vector3f massTrans)
{
    btMass BulletMass;
    PrepareCapsule(BulletMass, density, radius, length);
    return AddMass(BulletMass, matrix, massTrans);
}

void RigidBodyImp::PrepareCapsuleTotal(btMass& mass, float total_mass, float radius, float length) const
{
    //// direction: (1=x, 2=y, 3=z)
    //int direction = 3;
    mass.mass = btScalar ( total_mass );
    mass.transform = btTransform::getIdentity();
}

void RigidBodyImp::SetCapsuleTotal(float total_mass, float radius, float length)
{
    //dBodyID BulletBody = (dBodyID) bodyID;
    btMass BulletMass;
    PrepareCapsuleTotal(BulletMass, total_mass, radius, length);
    SetMassParameters(BulletMass.mass,Vector3f(BulletMass.transform.getOrigin().m_floats));
    //dBodySetMass(BulletBody, &ODEMass);
}

salt::Vector3f RigidBodyImp::AddCapsuleTotal(float total_mass, float radius, float length, const salt::Matrix& matrix, salt::Vector3f massTrans)
{
    btMass BulletMass;
    PrepareCapsuleTotal(BulletMass, total_mass, radius, length);
    return AddMass(BulletMass, matrix, massTrans);
}

Vector3f RigidBodyImp::GetVelocity() const
{
    dBodyID BulletBody = (dBodyID) btID;
    if(!BulletBody->isRigidBody){
		std::cerr << "WARNING (BulletRigidBodyImp): tried to get velocity from a CollisionObject" <<std::endl;
		return Vector3f(0.0f,0.0f,0.0f);
	}
	const btVector3 &vel = ((btRigidBody *)BulletBody->obj)->getLinearVelocity();
    return Vector3f(vel.getX(), vel.getY(), vel.getZ());
}

void RigidBodyImp::SetVelocity(const Vector3f& vel)
{
    dBodyID BulletBody = (dBodyID) btID;
    if(!BulletBody->isRigidBody){
		std::cerr << "WARNING (BulletRigidBodyImp): tried to set velocity on a CollisionObject" <<std::endl;
		return;
	}
	((btRigidBody *)BulletBody->obj)->setLinearVelocity( btVector3( btScalar(vel[0]), btScalar(vel[1]), btScalar(vel[2]) ) );
}

void RigidBodyImp::SetRotation(const Matrix& rot)
{
    //TODO: check whether to use MotionStates here instead of getWorldTransform()
    dBodyID BulletBody = (dBodyID) btID;
	if(!BulletBody->isRigidBody){
		std::cerr << "WARNING (BulletRigidBodyImp): tried to treat a CollisionObject as a RigidBody" <<std::endl;
		return;
	}
    btTransform BulletMatrix = ((btRigidBody *)BulletBody->obj)->getWorldTransform();
    GenericPhysicsMatrix& matrixRef = (GenericPhysicsMatrix&) BulletMatrix.getBasis();
    ConvertRotationMatrix(rot, matrixRef);
    BulletMatrix.setBasis((btMatrix3x3 &)matrixRef);
    ((btRigidBody *)BulletBody->obj)->setWorldTransform(BulletMatrix);
}

salt::Matrix RigidBodyImp::GetRotation() const
{
    //TODO: check whether to use MotionStates here instead of getWorldTransform()
    dBodyID BulletBody = (dBodyID) btID;
	if(!BulletBody->isRigidBody){
		std::cerr << "WARNING (BulletRigidBodyImp): tried to treat a CollisionObject as a RigidBody" <<std::endl;
		return salt::Matrix();
	}
    btMatrix3x3 BulletMatrix = ((btRigidBody *)BulletBody->obj)->getWorldTransform().getBasis();
    GenericPhysicsMatrix* matrixPtr = (GenericPhysicsMatrix*) &BulletMatrix;
    salt::Matrix rot;
    ConvertRotationMatrix(matrixPtr,rot);
    return rot;
}

Vector3f RigidBodyImp::GetLocalAngularVelocity() const
{
    //TODO: check whether motion states are better
    dBodyID BulletBody = (dBodyID) btID;
	if(!BulletBody->isRigidBody){
		std::cerr << "WARNING (BulletRigidBodyImp): tried to treat a CollisionObject as a RigidBody" <<std::endl;
		return Vector3f();
	}
    const btVector3& vel = ((btRigidBody *)BulletBody->obj)->getAngularVelocity();
    //Tranlate vector into local coordinates by mutliplying with rotation 
    //matrix translation is ignored because a vector is a direction and is
    //unaffected by movement
    btVector3 relvel = ((btRigidBody *)BulletBody->obj)->getWorldTransform().getBasis()*vel;
    return Vector3f(relvel.getX(), relvel.getY(), relvel.getZ());
}

Vector3f RigidBodyImp::GetAngularVelocity() const
{
    dBodyID BulletBody = (dBodyID) btID;
	if(!BulletBody->isRigidBody){
		std::cerr << "WARNING (BulletRigidBodyImp): tried to treat a CollisionObject as a RigidBody" <<std::endl;
		return Vector3f(0.0f,0.0f,0.0f);
	}
    const btVector3& vel = ((btRigidBody *)BulletBody->obj)->getAngularVelocity();
    return Vector3f(vel.getX(), vel.getY(), vel.getZ());
}

void RigidBodyImp::SetAngularVelocity(const Vector3f& vel)
{
    dBodyID BulletBody = (dBodyID) btID;
	if(!BulletBody->isRigidBody){
		std::cerr << "WARNING (BulletRigidBodyImp): tried to treat a CollisionObject as a RigidBody" <<std::endl;
		return;
	}
    ((btRigidBody *)BulletBody->obj)->setAngularVelocity(btVector3( 
                                                   btScalar(vel[0]),
                                                   btScalar(vel[1]),
                                                   btScalar(vel[2])
                                                 )
                                       );
}

salt::Matrix RigidBodyImp::GetSynchronisationMatrix()
{
    //TODO: check whether getWorldTransform or MotionState is better
    dBodyID BulletBody = (dBodyID) btID;
	if(!BulletBody->isRigidBody){
		std::cerr << "WARNING (BulletRigidBodyImp): tried to treat a CollisionObject as a RigidBody" <<std::endl;
		return salt::Matrix();
	}
    const btTransform& trans = ((btRigidBody *)BulletBody->obj)->getWorldTransform();
    const btVector3& pos = trans.getOrigin();
    const btMatrix3x3& rot = trans.getBasis();
    
    Matrix mat;
    //*rot[row] returns a const btScalar to the first element
    //in that row, & turns that into a pointer to cycle through
    //all 3 elements of the vector (x,y,z) with [].
    mat.m[0] = (&*rot[0])[0];
    mat.m[1] = (&*rot[1])[0];
    mat.m[2] = (&*rot[2])[0];
    mat.m[3] = 0;
    mat.m[4] = (&*rot[0])[1];
    mat.m[5] = (&*rot[1])[1];
    mat.m[6] = (&*rot[2])[1];
    mat.m[7] = 0;
    mat.m[8] =  (&*rot[0])[2];
    mat.m[9] =  (&*rot[1])[2];
    mat.m[10] = (&*rot[2])[2];
    mat.m[11] = 0;
    mat.m[12] = (&*pos)[0];
    mat.m[13] = (&*pos)[1];
    mat.m[14] = (&*pos)[2];
    mat.m[15] = 1;
    return mat;
}

RigidBody* RigidBodyImp::BodyGetData()
{
	if(!((dBodyID)btID)->isRigidBody){
		std::cerr << "WARNING (BulletRigidBodyImp): tried to treat a CollisionObject as a RigidBody" <<std::endl;
		return 0;
	}
	return (RigidBody*) userPointer;
    //return (RigidBody*) ((dBodyID)btID)->obj->getUserPointer();
}

void RigidBodyImp::AddForce(const Vector3f& force)
{
    dBodyID BulletBody = (dBodyID) btID;
    if(!BulletBody->isRigidBody){
		std::cerr << "WARNING (BulletRigidBodyImp): tried to treat a CollisionObject as a RigidBody" <<std::endl;
		return;
	}
	((btRigidBody *)BulletBody->obj)->applyCentralForce(btVector3( 
                                                btScalar(force[0]),
                                                btScalar(force[1]),
                                                btScalar(force[2])
                                                ));
}

Vector3f RigidBodyImp::GetForce() const
{
    dBodyID BulletBody = (dBodyID) btID;
    if(!BulletBody->isRigidBody){
		std::cerr << "WARNING (BulletRigidBodyImp): tried to treat a CollisionObject as a RigidBody" <<std::endl;
		return Vector3f();
	}
	const btVector3& f = ((btRigidBody *)BulletBody->obj)->getTotalForce();
    return Vector3f(f.getX(),f.getY(),f.getZ());
}

void RigidBodyImp::AddTorque(const Vector3f& torque)
{
    dBodyID BulletBody = (dBodyID) btID;
    if(!BulletBody->isRigidBody){
		std::cerr << "WARNING (BulletRigidBodyImp): tried to treat a CollisionObject as a RigidBody" <<std::endl;
		return;
	}
	((btRigidBody *)BulletBody->obj)->applyTorque(btVector3( 
                                          btScalar(torque[0]),
                                          btScalar(torque[1]),
                                          btScalar(torque[2])
                                          ));
}

void RigidBodyImp::SetPosition(const Vector3f& pos)
{
    //TODO: use MotionStates?
    dBodyID BulletBody = (dBodyID) btID;
    if(!BulletBody->isRigidBody){
		std::cerr << "WARNING (BulletRigidBodyImp): tried to treat a CollisionObject as a RigidBody" <<std::endl;
		return;
	}
	btTransform & trans = ((btRigidBody *)BulletBody->obj)->getWorldTransform();
    trans.setOrigin(btVector3( 
                             btScalar(pos[0]),
                             btScalar(pos[1]),
                             btScalar(pos[2])
                             ));
    ((btRigidBody *)BulletBody->obj)->setWorldTransform(trans);
    if(BulletBody->wrld)
	{
		//:TODO: handle collision bitmask
		BulletBody->wrld->removeRigidBody((btRigidBody *)BulletBody->obj);
		BulletBody->wrld->addRigidBody((btRigidBody *)BulletBody->obj);
		((btRigidBody *)BulletBody->obj)->activate(true);
		((btRigidBody *)BulletBody->obj)->setActivationState(DISABLE_DEACTIVATION);
	}
	
	//// the parent node will be updated in the next physics cycle

	//std::cout << "translated rigidbody"<<std::endl;
}

Vector3f RigidBodyImp::GetPosition() const
{
    dBodyID BulletBody = (dBodyID) btID;
    if(!BulletBody->isRigidBody){
		std::cerr << "WARNING (BulletRigidBodyImp): tried to treat a CollisionObject as a RigidBody" <<std::endl;
		return Vector3f();
	}
	const btVector3& pos = ((btRigidBody *)BulletBody->obj)->getWorldTransform().getOrigin();
    std::cout << "X:" << pos.getX() << "Y: " << pos.getY() << "Z:" << pos.getZ() << std::endl;
	return Vector3f(pos.getX(), pos.getY(), pos.getZ());
}

void RigidBodyImp::TranslateMass(const Vector3f& v)
{
    dBodyID BulletBody = (dBodyID) btID;
    if(!BulletBody->isRigidBody){
		std::cerr << "WARNING (BulletRigidBodyImp): tried to treat a CollisionObject as a RigidBody" <<std::endl;
		return;
	}
	//TODO: use native bullet calls instead of the abstract 
    // physicsserver ones
    float mass = GetMass();
    btMass BulletMass;
    SetMassParameters(BulletMass.mass,Vector3f(
		BulletMass.transform.getOrigin().m_floats));
    //dMass m;
    //dBodyGetMass(BulletBody, &m);
    //dMassTranslate(&m,v[0],v[1],v[2]);
}

btMass& RigidBodyImp::CreateMass(float mass, salt::Vector3f cVector)
{
    btMass BulletMass;
    BulletMass.mass = btScalar(0.0f);
    BulletMass.transform = btTransform::getIdentity();
    return BulletMass;
}

void RigidBodyImp::SetInertiaTensor(const salt::Matrix& tensor)
{
	//:TODO: there's no way to get write access to the inertia tensor
	// it may be possible to calculate the local inertia and set that
	// but the existance of this function is more than questionable
	// as it is already

	//dBodyID BulletBody = (dBodyID) BodyID;
	//((btRigidBody *)BulletBody->obj)->getInvInertiaTensorWorld(
    //btMass& ODEMass = (btMass&) mass;
    //ODEMass.I[i] = value;
}

//:TODO: get default mass from world or something
#undef DEFAULT_MASS