//===========================================================================
/*
This file is part of the CHAI 3D visualization and haptics libraries.
Copyright (C) 2003-2009 by CHAI 3D. All rights reserved.

This library is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License("GPL") version 2
as published by the Free Software Foundation.

For using the CHAI 3D libraries with software that can not be combined
with the GNU GPL, and for taking advantage of the additional benefits
of our support services, please contact CHAI 3D about acquiring a
Professional Edition License.

\author    <http://www.chai3d.org>
\author    Your Name!
\version   2.0.0 $Rev: 251 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CHapstickDeviceH
#define CHapstickDeviceH

#if defined(_ENABLE_HAPSTICK_DEVICE_SUPPORT)
//---------------------------------------------------------------------------
#include "devices/CGenericHapticDevice.h"
#include "HapstickDevice.h"
//---------------------------------------------------------------------------
#define C_SUCCESS 1
#define C_ERROR 0
//===========================================================================
/*!
\file       CHapstickDevice.h

\brief
<b> Devices </b> \n
Custom Haptic Device (Template).
*/
//===========================================================================

//===========================================================================
/*!
\class      CHapstickDevice
\ingroup    devices

\brief
CHapstickDevice provides the structure in which you can very easily
interface CHAI 3D to your custom haptic device. \n

Simply follow the comments and complete the gaps with your code.
Depending of the numbers of degrees of freedom of your device, not
all methods will need to be completed. For instaCHapstickDevicence, if your device
does not provide any rotation degrees of freedom, simply ignore
the getRotation() method. Default values will be returned correctly
if these are not implemented on your device

*/
//===========================================================================
class CHapstickDevice : public cGenericHapticDevice {
public:

	//-----------------------------------------------------------------------
	// CONSTRUCTOR & DESTRUCTOR:
	//-----------------------------------------------------------------------

	//! Constructor of CHapstickDevice.
	CHapstickDevice(unsigned int a_deviceNumber = 0);

	//! Destructor of CHapstickDevice.
	virtual ~CHapstickDevice();


	//-----------------------------------------------------------------------
	// METHODS:
	//-----------------------------------------------------------------------

	//! Open connection to haptic device (0 indicates success).
	int open();

	//! Close connection to haptic device (0 indicates success).
	int close();

	//! Initialize or calibrate haptic device (0 indicates success).
	int initialize(const bool a_resetEncoders = false);

	//! Returns the number of devices available from this class of device.
	unsigned int getNumDevices();

	//! Read the position of the device. Units are meters [m].
	int getPosition(cVector3d& a_position);

	//! Read the orientation frame of the device end-effector.
	int getRotation(cMatrix3d& a_rotation);

	//! Read the gripper angle in radian.
	int getGripperAngleRad(double& a_angle);

	//! Send a force [N] to the haptic device.
	int setForce(cVector3d& a_force);

	//! Send a torque [N*m] to the haptic device.
	int setTorque(cVector3d& a_torque);

	//! Send a torque [N*m] to the gripper.
	int setGripperTorque(double a_gripperTorque);

	//! Send a force [N] and a torque [N*m] and gripper torque [N*m] to the haptic device.
	int setForceAndTorqueAndGripper(cVector3d& a_force, cVector3d& a_torque, double a_gripperTorque);

	//! read the status of the user switch [\b true = \b ON / \b false = \b OFF].
	int getUserSwitch(int a_switchIndex, bool& a_status);


private:

	// pointer to device
	HapstickDevice *device;

protected:

	/********************************************************************
	If you need to declare any local variables or methods,
	you can do it here.
	*********************************************************************/

	

	// This method converts an input angle into a distance (meters)
	double getMyDevicePosition(double a_angle);

};

//---------------------------------------------------------------------------
#endif  // _ENABLE_HAPSTICK_DEVICE_SUPPORT
//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------