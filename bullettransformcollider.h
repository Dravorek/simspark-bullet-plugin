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

#ifndef BULLETTRANSFORMCOLLIDER_H
#define BULLETTRANSFORMCOLLIDER_H

#include "bulletcollider.h"
#include <oxygen/physicsserver/int/transformcolliderint.h>

class TransformColliderImp : public oxygen::TransformColliderInt, public ColliderImp
{
    /** TransformCollider encapsulates a transform geometry object
    that encapsulates another geom. It allows the encapsulated geom to
    be positioned and rotated arbitrarily with respect to its point of
    reference.

    Most geoms (like the sphere and box) have their point of
    reference corresponding to their center of mass, allowing them to
    be easily connected to dynamics objects. Transform objects give
    you more flexibility - for example, you can offset the center of a
    sphere, or rotate a cylinder so that its axis is something other
    than the default.

    Transform geoms are further used to create composite objects. As
    they allow multiple displaced geoms to be connected to one body.
    
    See physicsserver/int/transformcolliderint.h for documentation.
*/

public:    
    TransformColliderImp();
    void SetColliderParameters(int cleanup, int info);
    long CreateTransformCollider();
};

DECLARE_CLASS(TransformColliderImp);

#endif //BULLETCOLLIDER_H
