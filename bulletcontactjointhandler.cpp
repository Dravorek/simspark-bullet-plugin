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

#include "bulletcontactjointhandler.h"

using namespace oxygen;

ContactJointHandlerImp::ContactJointHandlerImp() : ColliderImp(){
		//std::cerr << "(ContactJointHandlerImp) ERROR called unimplemented constructor" << std::endl;
}

GenericSurfaceParameter* ContactJointHandlerImp::Initialize(){        
    //dSurfaceParameters* ODESurface = new dSurfaceParameters();

    //// set up default contact surface parameters
    //ODESurface->mode = dContactBounce;
    //ODESurface->mu = dInfinity;
    //ODESurface->bounce = 0.8f;
    //ODESurface->bounce_vel = 2.0f;

    //// In contactjointhandler_c.cpp, setContactSlip function, only
    //// accept one parameter, so two parameter is a mistake, and then 
    //// these two value will be not initialized, but slipe mode is open
    //ODESurface->slip1 = 5e-3;
    //ODESurface->slip2 = 5e-3;
    //
    //return (GenericSurfaceParameter*) ODESurface;
	//std::cerr << "(ContactJointHandlerImp) ERROR called unimplemented method: Initialize(" << std::endl;
    static GenericSurfaceParameter param;
    return &param;
}

BodyInt *ContactJointHandlerImp::RetrieveBody(ColliderInt *geomID){
    //dGeomID ODEGeom = (dGeomID) geomID;
    //dBodyID ODEBody = dGeomGetBody(ODEGeom);
    //return (long) ODEBody;
	//std::cerr << "(ContactJointHandlerImp) ERROR called unimplemented method: RetrieveBody(" << std::endl;
    return 0;
}

PhysicsObjectInt * 
ContactJointHandlerImp::CreateContactJoint(WorldInt *worldID, PhysicsObjectInt *jointGroupID, GenericContact& contact){
    //dWorldID ODEWorld = (dWorldID) worldID;
    //dJointGroupID ODEJointGroup = (dJointGroupID) jointGroupID;
    //dContact& ODEContact = (dContact&) contact;
    //
    //dJointID ODEJoint = dJointCreateContact
    //    (ODEWorld, ODEJointGroup, &ODEContact);
    //    
    //return (long) ODEJoint;
	//std::cerr << "(ContactJointHandlerImp) ERROR called unimplemented method: CreateContactJoint(" << std::endl;
    return 0;
}

void ContactJointHandlerImp::AttachContactJoint(PhysicsObjectInt *jointID, BodyInt *bodyID1, BodyInt *bodyID2){
    //dJointID ODEJoint = (dJointID) jointID;
    //dBodyID ODEBody1 = (dBodyID) bodyID1;
    //dBodyID ODEBody2 = (dBodyID) bodyID2;
    //
    //dJointAttach (ODEJoint, ODEBody1, ODEBody2);
	//std::cerr << "(ContactJointHandlerImp) ERROR called unimplemented method: AttachContactJoint(" << std::endl;
}

void 
ContactJointHandlerImp::CalcSurfaceParam(GenericContact& surface, 
                                         GenericSurfaceParameter& collideeParam,
                                         GenericSurfaceParameter* surfacePtr){
    //dContact& ODEContact = (dContact&) surface;
    //dSurfaceParameters& ODEParams = (dSurfaceParameters&) collideeParam;
    //dSurfaceParameters* ODESurface = (dSurfaceParameters*) surfacePtr;
    //
    //CalcSurfaceParamInternal(ODEContact.surface, ODEParams, ODESurface);
	//std::cerr << "(ContactJointHandlerImp) ERROR called unimplemented method: CalcSurfaceParam(" << std::endl;
}

float
ContactJointHandlerImp::MixValues(const float v1, const float v2, const int n) const
{
    //switch(n)
    //    {
    //    default:
    //    case 0:
    //        // undefined, default 0
    //        return 0.0f;

    //    case 1:
    //        // first one defined
    //        return v1;

    //    case 2:
    //        // second one defined
    //        return v2;

    //    case 3:
    //        // both defined, return average
    //        return (v1 + v2) / 2.0f;
    //    }
	//std::cerr << "(ContactJointHandlerImp) ERROR called unimplemented method: MixValues(" << std::endl;
    return 0.0f;
}

//void
//ContactJointHandlerImp::CalcSurfaceParamInternal(dSurfaceParameters& surface,
//                                                 const dSurfaceParameters& collideeParam,
//                                                 const dSurfaceParameters* surfacePtr)
//{
//    //// init surface
//    //surface.mode = 0;
//
//    //// calculate average mu; mu can be dInfinity, so first multiply with
//    //// 0.5 and the sum up to avoid a range error
//    //surface.mu = surfacePtr->mu*0.5f + collideeParam.mu*0.5f;
//
//    //// soft cfm
//    //const int nCfm =
//    //    ((surfacePtr->mode & dContactSoftCFM) ? 1:0) +
//    //    ((collideeParam.mode & dContactSoftCFM) ? 2:0);
//
//    //if (nCfm>0)
//    //    {
//    //        surface.soft_cfm = MixValues
//    //            (surfacePtr->soft_cfm, collideeParam.soft_cfm, nCfm);
//    //        surface.mode |= dContactSoftCFM;
//    //    }
//
//    //// soft_erp
//    //const int nErp =
//    //    ((surfacePtr->mode & dContactSoftERP) ? 1:0) +
//    //    ((collideeParam.mode & dContactSoftERP) ? 2:0);
//
//    //if (nErp>0)
//    //    {
//    //        surface.soft_erp = MixValues
//    //            (surfacePtr->soft_erp, collideeParam.soft_erp, nErp);
//    //        surface.mode |= dContactSoftERP;
//    //    }
//
//    //// bounce
//    //const int nBounce =
//    //    ((surfacePtr->mode & dContactBounce) ? 1:0) +
//    //    ((collideeParam.mode & dContactBounce) ? 2:0);
//
//    //if (nBounce>0)
//    //    {
//    //        surface.bounce = MixValues
//    //            (surfacePtr->bounce, collideeParam.bounce, nBounce);
//
//    //        surface.bounce_vel = MixValues
//    //            (surfacePtr->bounce_vel, collideeParam.bounce_vel, nBounce);
//
//    //        surface.mode |= dContactBounce;
//    //    }
//
//    //// slip1
//    //const int nSlip1 = 
//    //    ((surfacePtr->mode & dContactSlip1) ? 1 : 0) +
//    //    ((collideeParam.mode & dContactSlip1) ? 2 : 0);
//
//    //if (nSlip1 > 0)
//    //    {
//    //        surface.slip1 = MixValues
//    //            (surfacePtr->slip1, collideeParam.slip1, nSlip1);
//
//    //        surface.mode |= dContactSlip1;
//    //    }
//
//    //// slip2
//    //const int nSlip2 = 
//    //    ((surfacePtr->mode & dContactSlip2) ? 1 : 0) +
//    //    ((collideeParam.mode & dContactSlip2) ? 2 : 0);
//
//    //if (nSlip2 > 0)
//    //    {
//    //        surface.slip2 = MixValues
//    //            (surfacePtr->slip2, collideeParam.slip2, nSlip2);
//
//    //        surface.mode |= dContactSlip2;
//    //    }
//}
//
//void
//ContactJointHandlerImp::SetContactMode(int mode, bool set, dSurfaceParameters* ODESurface)
//{
//    //if (set)
//    //    {
//    //        ODESurface->mode |= mode;
//    //    } else
//    //        {
//    //            ODESurface->mode &= ~mode;
//    //        }
//}

int
ContactJointHandlerImp::GetContactMode(GenericSurfaceParameter* surface) const
{
    //dSurfaceParameters* ODESurface = (dSurfaceParameters*) surface;
    //return ODESurface->mode;
	//std::cerr << "(ContactJointHandlerImp) ERROR called unimplemented method: GetContactMode(" << std::endl;
    return 0;
}
    
void
ContactJointHandlerImp::SetContactBounceMode(bool set, GenericSurfaceParameter* surface)
{
    //dSurfaceParameters* ODESurface = (dSurfaceParameters*) surface;
    //SetContactMode(dContactBounce, set, ODESurface);
	//std::cerr << "(ContactJointHandlerImp) ERROR called unimplemented method: SetContactBounceMode(" << std::endl;
}

void
ContactJointHandlerImp::SetMinBounceVel(float vel, GenericSurfaceParameter* surface)
{
    //dSurfaceParameters* ODESurface = (dSurfaceParameters*) surface;
    //ODESurface->bounce_vel = std::max<float>(0.0f,vel);
	//std::cerr << "(ContactJointHandlerImp) ERROR called unimplemented method: SetMinBounceVel(" << std::endl;
}

float
ContactJointHandlerImp::GetMinBounceVel(GenericSurfaceParameter* surface) const
{
    //dSurfaceParameters* ODESurface = (dSurfaceParameters*) surface;
    //return ODESurface->bounce_vel;
	//std::cerr << "(ContactJointHandlerImp) ERROR called unimplemented method: GetMinBounceVel(" << std::endl;
    return 0.0f;
}

void
ContactJointHandlerImp::SetBounceValue(float bounce, GenericSurfaceParameter* surface)
{
    //dSurfaceParameters* ODESurface = (dSurfaceParameters*) surface;
    //ODESurface->bounce = std::max<float>(0.0f,bounce);
	//std::cerr << "(ContactJointHandlerImp) ERROR called unimplemented method: SetBounceValue(" << std::endl;
}

float
ContactJointHandlerImp::GetBounceValue(GenericSurfaceParameter* surface) const
{
    //dSurfaceParameters* ODESurface = (dSurfaceParameters*) surface;
    //return ODESurface->bounce;
	//std::cerr << "(ContactJointHandlerImp) ERROR called unimplemented method: GetBounceValue(" << std::endl;
    return 0.0f;
}

void
ContactJointHandlerImp::SetContactSoftERPMode(bool set, GenericSurfaceParameter* surface)
{
    //dSurfaceParameters* ODESurface = (dSurfaceParameters*) surface;
    //SetContactMode(dContactSoftERP, set, ODESurface);
	//std::cerr << "(ContactJointHandlerImp) ERROR called unimplemented method: SetContactSoftERPMode(" << std::endl;
}

void
ContactJointHandlerImp::SetContactSoftERP(float erp, GenericSurfaceParameter* surface)
{    
    //dSurfaceParameters* ODESurface = (dSurfaceParameters*) surface;
    //salt::gClamp(erp,0.0f,1.0f);
    //ODESurface->soft_erp = erp;
	//std::cerr << "(ContactJointHandlerImp) ERROR called unimplemented method: SetContactSoftERP(" << std::endl;
}

float
ContactJointHandlerImp::GetContactSoftERP(GenericSurfaceParameter* surface) const
{
    //dSurfaceParameters* ODESurface = (dSurfaceParameters*) surface;
    //return ODESurface->soft_erp;
	//std::cerr << "(ContactJointHandlerImp) ERROR called unimplemented method: GetContactSoftERP(" << std::endl;
    return 0.0f;
}

void
ContactJointHandlerImp::SetContactSoftCFMMode(bool set, GenericSurfaceParameter* surface)
{
    //dSurfaceParameters* ODESurface = (dSurfaceParameters*) surface;
    //SetContactMode(dContactSoftCFM, set, ODESurface);
	//std::cerr << "(ContactJointHandlerImp) ERROR called unimplemented method: SetContactSoftCFMMode(" << std::endl;
}

void
ContactJointHandlerImp::SetContactSoftCFM(float cfm, GenericSurfaceParameter* surface)
{
    //dSurfaceParameters* ODESurface = (dSurfaceParameters*) surface;
    //ODESurface->soft_cfm = std::max<float>(0.0f,cfm);
	//std::cerr << "(ContactJointHandlerImp) ERROR called unimplemented method: SetContactSoftCFM(" << std::endl;
}

float ContactJointHandlerImp::GetContactSoftCFM(GenericSurfaceParameter* surface) const
{
    //dSurfaceParameters* ODESurface = (dSurfaceParameters*) surface;
    //return ODESurface->soft_cfm;
	//std::cerr << "(ContactJointHandlerImp) ERROR called unimplemented method: GetContactSoftCFM(" << std::endl;
    return 0.0f;
}

void ContactJointHandlerImp::SetContactSlipMode (bool set, GenericSurfaceParameter* surface)
{
    //dSurfaceParameters* ODESurface = (dSurfaceParameters*) surface;
    //SetContactMode(dContactSlip1,set,ODESurface);
    //SetContactMode(dContactSlip2,set,ODESurface);
	//std::cerr << "(ContactJointHandlerImp) ERROR called unimplemented method: SetContactSlipMode (" << std::endl;
}

void ContactJointHandlerImp::SetContactSlip(float slip, GenericSurfaceParameter* surface)
{
    //dSurfaceParameters* ODESurface = (dSurfaceParameters*) surface;
    //ODESurface->slip1 = slip;
    //ODESurface->slip2 = slip;
	//std::cerr << "(ContactJointHandlerImp) ERROR called unimplemented method: SetContactSlip(" << std::endl;
}

float
ContactJointHandlerImp::GetContactSlip1(GenericSurfaceParameter* surface) const
{
    //dSurfaceParameters* ODESurface = (dSurfaceParameters*) surface;
    //return ODESurface->slip1;
	//std::cerr << "(ContactJointHandlerImp) ERROR called unimplemented method: GetContactSlip1(" << std::endl;
    return 0.0f;
}

float
ContactJointHandlerImp::GetContactSlip2(GenericSurfaceParameter* surface) const
{
    //dSurfaceParameters* ODESurface = (dSurfaceParameters*) surface;
    //return ODESurface->slip2;
	//std::cerr << "(ContactJointHandlerImp) ERROR called unimplemented method: GetContactSlip2(" << std::endl;
    return 0.0f;
}

void ContactJointHandlerImp::SetContactMu(float mu, GenericSurfaceParameter* surface)
{
    //dSurfaceParameters* ODESurface = (dSurfaceParameters*) surface;
    //ODESurface->mu = mu;
	//std::cerr << "(ContactJointHandlerImp) ERROR called unimplemented method: SetContactMu(" << std::endl;
}

float ContactJointHandlerImp::GetContactMu(GenericSurfaceParameter* surface) const
{
    //dSurfaceParameters* ODESurface = (dSurfaceParameters*) surface;
    //return ODESurface->mu;
	//std::cerr << "(ContactJointHandlerImp) ERROR called unimplemented method: GetContactMu(" << std::endl;
    return 0.0f;
}
