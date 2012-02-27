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
#ifndef BULLETRIGIDBODY_H
#define BULLETRIGIDBODY_H

#include "bulletbody.h"
#include <oxygen/physicsserver/int/rigidbodyint.h>

struct btMass{
    btTransform transform;
    btScalar mass;
};
//typedef btVector3 btMass;

class RigidBodyImp : public oxygen::RigidBodyInt, public BodyImp
{
/** RigidBody encapsulates a rigid body object. A rigid body has
    various properties from the point of view of the simulation. Some
    properties change over time: position, linear velocity,
    orientation and angular velocity. Other body properties are
    usually constant over time: Mass, Center of mass and Inertia
    matrix (mass distribution matrix)
    
    See physicsserver/int/rigidbodyint.h for documentation.
*/
public:
    RigidBodyImp();
    
    oxygen::RigidBody* GetBodyPointer();
    void Enable();
    void Disable();
    bool IsEnabled() const;
    void UseGravity(bool f);
    bool UsesGravity() const;
    void SetMass(float mass);
    void SetMassParameters(const float mass,
                           const salt::Vector3f& center_of_mass);
    float GetMass() const;
    void SetSphere(float density, float radius);
    salt::Vector3f AddSphere(float density, float radius, const salt::Matrix& matrix,
                             salt::Vector3f massTrans);
    void SetSphereTotal(float total_mass, float radius);
    salt::Vector3f AddSphereTotal(float total_mass, float radius, const salt::Matrix& matrix,
                                  salt::Vector3f massTrans);
    void SetBox(float density, const salt::Vector3f& size);
    salt::Vector3f AddBox(float density, const salt::Vector3f& size, const salt::Matrix& matrix,
                          salt::Vector3f massTrans);
    void SetBoxTotal(float total_mass, const salt::Vector3f& size);
    salt::Vector3f AddBoxTotal(float total_mass, const salt::Vector3f& size, const salt::Matrix& matrix,
                               salt::Vector3f massTrans);
    void SetCylinder(float density, float radius, float length);
    salt::Vector3f AddCylinder(float density, float radius, float length, const salt::Matrix& matrix,
                               salt::Vector3f massTrans);
    void SetCylinderTotal(float total_mass, float radius, float length);
    salt::Vector3f AddCylinderTotal(float total_mass, float radius, float length, const salt::Matrix& matrix,
                                    salt::Vector3f massTrans);
    void SetCapsule(float density, float radius, float length);
    salt::Vector3f AddCapsule(float density, float radius, float length, const salt::Matrix& matrix,
                              salt::Vector3f massTrans);
    void SetCapsuleTotal(float total_mass, float radius, float length);
    salt::Vector3f AddCapsuleTotal(float total_mass, float radius, float length, const salt::Matrix& matrix,
                                   salt::Vector3f massTrans);
    void TranslateMass(const salt::Vector3f& v);
    salt::Vector3f GetVelocity() const;
    void SetVelocity(const salt::Vector3f& vel);
    void SetRotation(const salt::Matrix& rot);
    salt::Matrix GetRotation() const;
    salt::Vector3f GetLocalAngularVelocity() const;
    salt::Vector3f GetAngularVelocity() const;
    void SetAngularVelocity(const salt::Vector3f& vel);
    void AddForce(const salt::Vector3f& force);
    salt::Vector3f GetForce() const;
    void AddTorque(const salt::Vector3f& torque);
    void SetPosition(const salt::Vector3f& pos);
    salt::Vector3f GetPosition() const;
    void DestroyRigidBody();
    salt::Matrix GetSynchronisationMatrix();
    void BodySetData(oxygen::RigidBody* rb);
    oxygen::RigidBody* BodyGetData();    
    long CreateBody(oxygen::WorldInt *worldID);
    void SetInertiaTensor(const salt::Matrix& tensor);
    
	void *userPointer;
protected:
    //These methods are only called internally and are not declared in the interface.
    btMass& CreateMass(float mass, salt::Vector3f cVector);
    salt::Vector3f AddMass(const btMass& mass, const salt::Matrix& matrix, salt::Vector3f massTrans);
    
    bool CheckCompoundStatus( bool prepareAddition);
    bool CheckShapeHasCollider();
    /** sets up an ode mass struct representing a box of the given
        size and total_mass
    */
    void PrepareBoxTotal(btMass& mass, float total_mass, const salt::Vector3f& size) const;
    
    /** sets up an ode mass struct representing a box of the given
        density and size
    */
    void PrepareBox(btMass& mass, float density, const salt::Vector3f& size) const;
    
    /** sets up an ode mass struct representing a sphere of the given
        density and radius
    */
    void PrepareSphere(btMass& mass, float density, float radius) const;   
    
    /** sets up an ode mass struct representing a sphere of the given
        radius and total_mass
    */
    void PrepareSphereTotal(btMass& mass, float total_mass, float radius) const;
    
    /** sets up an ode mass struct representing a flat-ended cylinder
        of the given parameters and density, with the center of mass
        at (0,0,0) relative to the body. The radius of the cylinder is
        radius. The length of the cylinder is length. The cylinder's
        long axis is oriented along the body's z axis.
     */
    void PrepareCylinder(btMass& mass, float density, float radius, float length) const;
    
    /** sets up an ode mass struct representing a flat-ended cylinder
        of the given parameters and total mass, with the center of
        mass at (0,0,0) relative to the body. The radius of the
        cylinder is radius. The length of the cylinder is length. The
        cylinder's long axis is oriented along the body's z axis.
     */
    void PrepareCylinderTotal(btMass& mass, float total_mass, float radius, float length) const;
    
    /** sets up an ode mass struct representing a capsule of
        the given parameters and density, with the center of mass at
        (0,0,0) relative to the body. The radius of the capsule (and
        the spherical cap) is radius. The length of the capsule (not
        counting the spherical cap) is length. The capsule's long axis
        is oriented along the body's z axis.
    */
    void PrepareCapsule(btMass& mass, float density, float radius, float length) const;
    
    /** sets up an ode mass struct representing a capsule of
        the given parameters and total mass, with the center of mass at
        (0,0,0) relative to the body. The radius of the capsule (and
        the spherical cap) is radius. The length of the capsule (not
        counting the spherical cap) is length. The capsule's long axis
        is oriented along the body's z axis.
    */
    void PrepareCapsuleTotal(btMass& mass, float total_mass, float radius, float length) const;
};

DECLARE_CLASS(RigidBodyImp);

#endif //BULLETRIGIDBODY_H
