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
#ifndef BULLETJOINT_H
#define BULLETJOINT_H

#include "bulletphysicsobject.h"
#include <oxygen/physicsserver/genericphysicsobjects.h>
#include <oxygen/physicsserver/int/jointint.h>

class JointImp : public oxygen::JointInt, public PhysicsObjectImp
{
    /** See physicsserver/int/jointint.h for documentation */

public:
    JointImp();

    oxygen::Joint* GetJoint();
    bool AreConnected(oxygen::BodyInt *bodyID1, oxygen::BodyInt *bodyID2);
    bool AreConnectedExcluding(oxygen::BodyInt *bodyID1, oxygen::BodyInt *bodyID2, int joint_type);
    virtual void DestroyJoint(
                              boost::shared_ptr<oxygen::GenericJointFeedback> feedback);
    virtual void Attach(oxygen::BodyInt *bodyID1, oxygen::BodyInt *bodyID2);
    int GetType() const;
    oxygen::BodyInt *GetBodyID(int idx);
    void EnableFeedback(bool enable ,
                        boost::shared_ptr<oxygen::GenericJointFeedback>& feedback);
    bool FeedbackEnabled() const;
    salt::Vector3f GetFeedbackForce(int idx,
                                    boost::shared_ptr<oxygen::GenericJointFeedback> feedback) const;
    salt::Vector3f GetFeedbackTorque(int idx,
                                     boost::shared_ptr<oxygen::GenericJointFeedback> feedback) const;
    void SetFudgeFactor(int idx, float fudge_factor);
    float GetFudgeFactor(int idx) const;
    void SetBounce(int idx, float bounce);
    float GetBounce(int idx) const;
    void SetLowStopDeg(int idx, float deg);
    float GetLowStopDeg(int idx) const;
    void SetHighStopDeg(int idx, float deg);
    float GetHighStopDeg(int idx) const;
    void SetLowStopPos(int idx, float deg);
    float GetLowStopPos(int idx) const;
    void SetHighStopPos(int idx, float deg);
    float GetHighStopPos(int idx) const;
    void SetCFM(int idx, float cfm);
    float GetCFM(int idx) const;
    void SetStopCFM(int idx, float cfm);
    float GetStopCFM(int idx) const;
    void SetStopERP(int idx, float erp);
    float GetStopERP(int idx) const;
    void SetSuspensionERP(int idx, float erp);
    float GetSuspensionERP(int idx) const;
    void SetSuspensionCFM(int idx, float cfm);
    float GetSuspensionCFM(int idx) const;
    void SetLinearMotorVelocity(int idx, float vel);
    float GetLinearMotorVelocity(int idx) const;
    void SetAngularMotorVelocity(int idx, float deg);
    float GetAngularMotorVelocity(int idx) const;
    void SetMaxMotorForce(int idx, float f);
    float GetMaxMotorForce(int idx) const;
    void SetParameter(int parameter, float value);
    float GetParameter(int parameter) const;
    void OnLink( oxygen::Joint* joint);

	float rad2deg(float rad) const;
	float deg2rad(float deg) const;

	void *userPointer;
	btJointWrapper *jointID;
protected:
};

DECLARE_CLASS(JointImp);

#endif //ODEJOINT_H
