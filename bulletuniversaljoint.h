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

#ifndef BULLETUNIVERSALJOINT_H
#define BULLETUNIVERSALJOINT_H

#include "bulletgeneric6dofjoint.h"
#include <oxygen/physicsserver/int/universaljointint.h>

class UniversalJointImp : public oxygen::UniversalJointInt, public Generic6DOFJointImp
{
    /** See physicsserver/int/universaljointint.h for documentation */
    
public:
    UniversalJointImp();
    long CreateUniversalJoint(oxygen::WorldInt *world);
    void SetAnchor(const salt::Vector3f& anchor);
    salt::Vector3f GetAnchor1();
    salt::Vector3f GetAnchor2();
    void SetAxis1(const salt::Vector3f& axis);
    void SetAxis2(const salt::Vector3f& axis);
    salt::Vector3f GetAxis1() const;
    salt::Vector3f GetAxis2() const;
    float GetAngle1() const;
    float GetAngle2() const;
    float GetAngleRate1() const;
    float GetAngleRate2() const;
	void Attach(oxygen::BodyInt *bodyID1, oxygen::BodyInt *bodyID2);
};

DECLARE_CLASS(UniversalJointImp);

#endif //BULLETUNIVERSALJOINT_H
