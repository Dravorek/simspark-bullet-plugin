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

#ifndef BULLETHINGEJOINT_H
#define BULLETHINGEJOINT_H

#include "bulletgeneric6dofjoint.h"
#include <oxygen/physicsserver/int/hingejointint.h>

class HingeJointImp : public oxygen::HingeJointInt, public Generic6DOFJointImp
{
    /** See physicsserver/int/hingejointint.h for documentation */
    
public:
    HingeJointImp();
    long CreateHingeJoint(oxygen::WorldInt *worldID);
    void SetAnchor(const salt::Vector3f& anchor);
    salt::Vector3f GetAnchor1();
    salt::Vector3f GetAnchor2();
    void SetAxis(const salt::Vector3f& axis);
    salt::Vector3f GetAxis();
    float GetAngle() const;
    float GetAngleRate() const;
    float GetTorque() const;
	virtual void Attach(oxygen::BodyInt *bodyID1, oxygen::BodyInt *bodyID2);

	void SetAngularMotorVelocity(int idx, float deg);
    float GetAngularMotorVelocity(int idx) const;
    void SetMaxMotorForce(int idx, float f);
    float GetMaxMotorForce(int idx) const;

	void SetLowStopDeg(int idx, float deg);

	float GetLowStopDeg(int idx) const;

	void SetHighStopDeg(int idx, float deg);

	float GetHighStopDeg(int idx) const;

	
    void SetBounce(int idx, float bounce);
    float GetBounce(int idx) const;

	void SetParameter(int parameter, float value);
    float GetParameter(int parameter) const;

	void SetCFM(int idx, float cfm);
    float GetCFM(int idx) const;
    void SetStopCFM(int idx, float cfm);
    float GetStopCFM(int idx) const;
    void SetStopERP(int idx, float erp);
    float GetStopERP(int idx) const;
    
private:
    //dJointFeedback mFeedback;

};

DECLARE_CLASS(HingeJointImp);

#endif //BULLETHINGEJOINT_H
