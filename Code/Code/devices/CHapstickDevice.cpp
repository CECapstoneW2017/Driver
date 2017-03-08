//==============================================================================
/*
Software License Agreement (BSD License)
Copyright (c) 2003-2016, CHAI3D.
(www.chai3d.org)

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above
copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided
with the distribution.

* Neither the name of CHAI3D nor the names of its contributors may
be used to endorse or promote products derived from this software
without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

\author    <http://www.chai3d.org>
\author    Your name, institution, or company name.
\version   3.2.0 $Rev: 1869 $
*/
//==============================================================================


//------------------------------------------------------------------------------
#include "extras/CGlobals.h"
#include "devices/CHapstickDevice.h"
//------------------------------------------------------------------------------
#if defined(_ENABLE_HAPSTICK_DEVICE_SUPPORT)
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
/*
INSTRUCTION TO IMPLEMENT YOUR OWN CUSTOM DEVICE:

Please review header file CHapstickDevice.h for some initial
guidelines about how to implement your own haptic device using this
template.

When ready, simply completed the next 11 documented steps described here
below.
*/
////////////////////////////////////////////////////////////////////////////////

//==============================================================================
/*!
Constructor of CHapstickDevice.
*/
//==============================================================================
CHapstickDevice::CHapstickDevice(unsigned int a_deviceNumber)
{
	// the connection to your device has not yet been established.
	m_systemReady = false;


	////////////////////////////////////////////////////////////////////////////
	/*
	STEP 1:

	Here you should define the specifications of your device.
	These values only need to be estimates. Since haptic devices often perform
	differently depending of their configuration withing their workspace,
	simply use average values.
	*/
	////////////////////////////////////////////////////////////////////////////


	m_specifications.m_manufacturerName = "Kettering";
	m_specifications.m_modelName = "Hapstick";
	m_specifications.m_maxForce = 8.0; //Our force = ? 3.0; // [N]
	m_specifications.m_maxForceStiffness = 3000.0; //Our force = ?// [N/m]
	m_specifications.m_maxTorque = 0.0; // [N*m]
	m_specifications.m_maxTorqueStiffness = 0.0; // [N*m/Rad]
	m_specifications.m_maxGripperTorque = 0.0; // [N]
	m_specifications.m_maxGripperTorqueStiffness = 0.0; // [N*m/m]0
	m_specifications.m_maxLinearDamping = 20.0; //Our force = ?// [N/(m/s)]
	m_specifications.m_workspaceRadius = .3;// = 0.0762; // [m]
	m_specifications.m_sensedPosition = true;
	m_specifications.m_sensedRotation = false;
	m_specifications.m_sensedGripper = false;
	m_specifications.m_actuatedPosition = true;
	m_specifications.m_actuatedRotation = false;
	m_specifications.m_actuatedGripper = false;
	m_specifications.m_leftHand = true;
	m_specifications.m_rightHand = true;

	//--------------------------------------------------------------------------
	// NAME:
	//--------------------------------------------------------------------------
	/*
	// haptic device model (see file "CGenericHapticDevice.h")
	m_specifications.m_model                         = C_HAPTIC_DEVICE_CUSTOM;

	// name of the device manufacturer, research lab, university.
	m_specifications.m_manufacturerName              = "Kettering University";

	// name of your device
	m_specifications.m_modelName                     = "Hapstick";


	//--------------------------------------------------------------------------
	// CHARACTERISTICS: (The following values must be positive or equal to zero)
	//--------------------------------------------------------------------------

	// the maximum force [N] the device can produce along the x,y,z axis.
	m_specifications.m_maxLinearForce                = 3.0;     // [N]

	// the maximum amount of torque your device can provide arround its
	// rotation degrees of freedom.
	m_specifications.m_maxAngularTorque              = 0;     // [N*m]


	// the maximum amount of torque which can be provided by your gripper
	m_specifications.m_maxGripperForce                = 0;     // [N]

	// the maximum closed loop linear stiffness in [N/m] along the x,y,z axis
	m_specifications.m_maxLinearStiffness             = 1000.0; // [N/m]

	// the maximum amount of angular stiffness
	m_specifications.m_maxAngularStiffness            = 0;    // [N*m/Rad]

	// the maximum amount of stiffness supported by the gripper
	m_specifications.m_maxGripperLinearStiffness      = 0;   // [N*m]

	// the radius of the physical workspace of the device (x,y,z axis)
	m_specifications.m_workspaceRadius                = 0.2;     // [m]

	// the maximum opening angle of the gripper
	m_specifications.m_gripperMaxAngleRad             = 0;

	*/
	////////////////////////////////////////////////////////////////////////////
	/*
	DAMPING PROPERTIES:

	Start with small values as damping terms can be high;y sensitive to
	the quality of your velocity signal and the spatial resolution of your
	device. Try gradually increasing the values by using example "01-devices"
	and by enabling viscosity with key command "2".
	*/
	////////////////////////////////////////////////////////////////////////////
	/*
	// Maximum recommended linear damping factor Kv
	m_specifications.m_maxLinearDamping             = 20.0;   // [N/(m/s)]

	//! Maximum recommended angular damping factor Kv (if actuated torques are available)
	m_specifications.m_maxAngularDamping            = 0.0;    // [N*m/(Rad/s)]

	//! Maximum recommended angular damping factor Kv for the force gripper. (if actuated gripper is available)
	m_specifications.m_maxGripperAngularDamping     = 0.0;    // [N*m/(Rad/s)]


	//--------------------------------------------------------------------------
	// CHARACTERISTICS: (The following are of boolean type: (true or false)
	//--------------------------------------------------------------------------

	// does your device provide sensed position (x,y,z axis)?
	m_specifications.m_sensedPosition                = true;

	// does your device provide sensed rotations (i.e stylus)?
	m_specifications.m_sensedRotation                = false;

	// does your device provide a gripper which can be sensed?
	m_specifications.m_sensedGripper                 = false;

	// is you device actuated on the translation degrees of freedom?
	m_specifications.m_actuatedPosition              = true;

	// is your device actuated on the rotation degrees of freedom?
	m_specifications.m_actuatedRotation              = false;

	// is the gripper of your device actuated?
	m_specifications.m_actuatedGripper               = false;

	// can the device be used with the left hand?
	m_specifications.m_leftHand                      = true;

	// can the device be used with the right hand?
	m_specifications.m_rightHand                     = true;
	*/

	////////////////////////////////////////////////////////////////////////////
	/*
	STEP 2:

	Here, you shall  implement code which tells the application if your
	device is actually connected to your computer and can be accessed.
	In practice this may be consist in checking if your I/O board
	is active or if your drivers are available.

	If your device can be accessed, set:
	m_systemAvailable = true;

	Otherwise set:
	m_systemAvailable = false;

	Your actual code may look like:

	bool result = checkIfMyDeviceIsAvailable()
	m_systemAvailable = result;

	If want to support multiple devices, using the method argument
	a_deviceNumber to know which device to setup
	*/
	////////////////////////////////////////////////////////////////////////////
	this->device = new HapstickLib();

	if (device != NULL){
		m_systemAvailable = true; // this value should become 'true' when the device is available.
	}
	else
	{
		m_systemAvailable = false; // this value should become 'true' when the device is available.
	}

}


//==============================================================================
/*!
Destructor of CHapstickDevice.
*/
//==============================================================================
CHapstickDevice::~CHapstickDevice()
{
	// close connection to device
	if (m_systemReady)
	{
		close();
	}
}


//==============================================================================
/*!
This method opens a connection to your device.

\return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
int CHapstickDevice::open()
{
	// check if the system is available
	if (!m_systemAvailable) return (C_ERROR);

	// if system is already opened then return
	if (m_systemReady) return (C_ERROR);

	////////////////////////////////////////////////////////////////////////////
	/*
	STEP 3:

	Here you shall implement to open a connection to your
	device. This may include opening a connection to an interface board
	for instance or a USB port.

	If the connection succeeds, set the variable 'result' to true.
	otherwise, set the variable 'result' to false.

	Verify that your device is calibrated. If your device
	needs calibration then call method calibrate() for wich you will
	provide code in STEP 5 further below.
	*/
	////////////////////////////////////////////////////////////////////////////

	int result = C_ERROR; // this value will need to become "C_SUCCESS" for the device to be marked as ready.

	// *** INSERT YOUR CODE HERE ***
	result = device->open(1);


	// update device status
	if (result)
	{
		m_systemReady = true;
		return (C_SUCCESS);
	}
	else
	{
		m_systemReady = false;
		return (C_ERROR);
	}
}


//==============================================================================
/*!
This method closes the connection to your device.

\return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
int CHapstickDevice::close()
{
	// check if the system has been opened previously
	if (!m_systemReady) return (C_ERROR);

	////////////////////////////////////////////////////////////////////////////
	/*
	STEP 4:

	Here you shall implement code that closes the connection to your
	device.

	If the operation fails, simply set the variable 'result' to C_ERROR   .
	If the connection succeeds, set the variable 'result' to C_SUCCESS.
	*/
	////////////////////////////////////////////////////////////////////////////

	int result = C_SUCCESS; // if the operation fails, set value to C_ERROR.

	// *** INSERT YOUR CODE HERE ***
	result = device->close();

	// update status
	m_systemReady = false;

	return (result);
}


//==============================================================================
/*!
This method calibrates your device.

\return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
int CHapstickDevice::initialize(bool a_resetEncoders)
{
	// check if the device is read. See step 3.
	if (!m_systemReady) return (C_ERROR);

	////////////////////////////////////////////////////////////////////////////
	/*
	STEP 5:

	Here you shall implement code that handles a calibration procedure of the
	device. In practice this may include initializing the registers of the
	encoder counters for instance.

	If the device is already calibrated and  a_forceCalibration == false,
	the method may immediately return without further action.
	If a_forceCalibration == true, then the calibrartion procedure
	shall be executed even if the device has already been calibrated.

	If the calibration procedure succeeds, the method returns C_SUCCESS,
	otherwise return C_ERROR.
	*/
	////////////////////////////////////////////////////////////////////////////

	int result = C_SUCCESS;

	// *** INSERT YOUR CODE HERE ***

	// error = calibrateMyDevice()

	return result;
}


//==============================================================================
/*!
This method returns the number of devices available from this class of device.

\return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
unsigned int CHapstickDevice::getNumDevices()
{
	////////////////////////////////////////////////////////////////////////////
	/*
	STEP 6:

	Here you shall implement code that returns the number of available
	haptic devices of type "CHapstickDevice" which are currently connected
	to your computer.

	In practice you will often have either 0 or 1 device. In which case
	the code here below is already implemented for you.

	If you have support more than 1 devices connected at the same time,
	then simply modify the code accordingly so that "numberOfDevices" takes
	the correct value.
	*/
	////////////////////////////////////////////////////////////////////////////

	// *** INSERT YOUR CODE HERE, MODIFY CODE below ACCORDINGLY ***

	int result = C_ERROR;
	int numberOfDevices = 0;  // At least set to 1 if a device is available.

	result = device->getDeviceCount(&numberOfDevices);

	return (numberOfDevices);
}


//==============================================================================
/*!
This method returns the position of your device. Units are meters [m].

\param   a_position  Return value.

\return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
int CHapstickDevice::getPosition(cVector3d& a_position) {
	// check if the device is read. See step 3.
	if (!m_systemReady) return (C_ERROR);

	/*
	Note:
	For consistency, units must be in meters.
	If your device is located in front of you, the x-axis is pointing
	towards you (the operator). The y-axis points towards your right
	hand side and the z-axis points up towards the sky.
	*/

	int result = C_SUCCESS;
	double xAngle, yAngle, zAngle;

	// *** INSERT YOUR CODE HERE, MODIFY CODE below ACCORDINGLY ***
	result = device->hapstickGetPosition(&xAngle, &yAngle, &zAngle);

	//add back later
	double x = (xAngle / 360) * 2 * PI * JOYSTICK_LENGTH;
	double y = (yAngle / 360) * 2 * PI * JOYSTICK_LENGTH;
	double z = (zAngle / 360) * 2 * PI * JOYSTICK_LENGTH;;

	// store new position values
	a_position.set(x, y, z);

	// estimate linear velocity
	estimateLinearVelocity(a_position);

	// exit
	return (result);
}


/*
* this method estimates linear velocity of the Hapstick device
*/
int CHapstickDevice::getLinearVelocity(cVector3d& a_linearVelocity) {

	a_linearVelocity = m_linearVelocity;

	return 1;
}


//==============================================================================
/*!
This method returns the orientation frame of your device end-effector

\param   a_rotation  Return value.

\return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================


int CHapstickDevice::getRotation(cMatrix3d& a_rotation)
{
	// check if the device is read. See step 3.
	if (!m_systemReady) return (C_ERROR);

	////////////////////////////////////////////////////////////////////////////
	/*
	STEP 8:

	Here you shall implement code which reads the orientation frame from
	your haptic device. The orientation frame is expressed by a 3x3
	rotation matrix. The 1st column of this matrix corresponds to the
	x-axis, the 2nd column to the y-axis and the 3rd column to the z-axis.
	The length of each column vector should be of length 1 and vectors need
	to be orthogonal to each other.

	Note:
	If your device is located in front of you, the x-axis is pointing
	towards you (the operator). The y-axis points towards your right
	hand side and the z-axis points up towards the sky.

	If your device has a stylus, make sure that you set the reference frame
	so that the x-axis corresponds to the axis of the stylus.
	*/
	////////////////////////////////////////////////////////////////////////////

	int result = C_SUCCESS;

	// variables that describe the rotation matrix
	double r00, r01, r02, r10, r11, r12, r20, r21, r22;
	cMatrix3d frame;
	frame.identity();

	// *** INSERT YOUR CODE HERE, MODIFY CODE below ACCORDINGLY ***

	// if the device does not provide any rotation capabilities 
	// set the rotation matrix equal to the identity matrix.
	r00 = 1.0;  r01 = 0.0;  r02 = 0.0;
	r10 = 0.0;  r11 = 1.0;  r12 = 0.0;
	r20 = 0.0;  r21 = 0.0;  r22 = 1.0;

	frame.set(r00, r01, r02, r10, r11, r12, r20, r21, r22);

	// store new rotation matrix
	a_rotation = frame;

	// estimate angular velocity
	estimateAngularVelocity(a_rotation);

	// exit
	return (result);
}

//===========================================================================
/*!
Send a force [N] to your haptic device

\fn     int cMyCustomDevice::setForce(cVector3d& a_force)
\param  a_force  Force command to be applied to device.
\return Return 0 if no error occurred.
*/
//===========================================================================
int CHapstickDevice::setForce(cVector3d& a_force)
{
	// temp variable
	int result = 0;

	// store new force value.
	m_prevForce = a_force;


	/************************************************************************
	STEP 9:
	Here you may implement code which sends a force command (fx,fy,fz)
	to your haptic device.
	If the operation fails return an error code.

	Note:
	For consistency, units must be in Newtons.
	If your device is located in front of you, the x-axis is pointing
	towards you (the operator). The y-axis points towards your right
	hand side and the z-axis points up towards the sky.

	For instance: if the force = (1,0,0), the device will move towards
	the operator, if the force = (0,0,1), the device will move upwards.

	*************************************************************************/


	// *** INSERT YOUR CODE HERE ***
	double conversionFactor = MAX_SIGNAL_FORCE / m_specifications.m_maxForce;

	int fx = a_force.x * conversionFactor;
	int fy = a_force.y * conversionFactor;
	int fz = a_force.z * conversionFactor;
	
	result = device->hapstickSetForce(&fx, &fy, &fz);

	// exit
	return (result);
}


//===========================================================================
/*!
Send a torque [N*m] to the haptic device

\fn     int cMyCustomDevice::setTorque(cVector3d& a_torque)
\param  a_torque Force command to be applied to device.
\return Return 0 if no error occurred.
*/
//===========================================================================
int CHapstickDevice::setTorque(cVector3d& a_torque)
{
	// temp variable
	int error = 0;

	// store new torque values
	m_prevTorque = a_torque;

	/************************************************************************
	STEP 10:
	Here you may implement code which sends a torque command (Tx,Ty,Tz)
	to your haptic device. This would be implemented if you have
	a haptic device with an active stylus for instance.
	If the operation fails return an error code.

	Note:
	For consistency, units must be in Newton meters.
	A torque (1,0,0) would rotate counter clock-wise around the x-axis.
	*************************************************************************/

	// *** INSERT YOUR CODE HERE ***

	// double tx = a_torque.x;
	// double ty = a_torque.y;
	// double tz = a_torque.z;

	// exit
	return (error);
}

//==============================================================================
/*!
This method returns status of all user switches
[__true__ = __ON__ / __false__ = __OFF__].

\param  a_userSwitches  Return the 32-bit binary mask of the device buttons.

\return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
int CHapstickDevice::getUserSwitch(int a_switchIndex, bool& a_status)
{
	// check if the device is read. See step 3.
	if (!m_systemReady) return (C_ERROR);

	////////////////////////////////////////////////////////////////////////////
	/*
	STEP 11:

	Here you shall implement code that reads the status all user switches
	on your device. For each user switch, set the associated bit on variable
	a_userSwitches. If your device only has one user switch, then set
	a_userSwitches to 1, when the user switch is engaged, and 0 otherwise.
	*/
	////////////////////////////////////////////////////////////////////////////

	// *** INSERT YOUR CODE HERE ***
	a_switchIndex = 0;

	return (C_SUCCESS);
}

//------------------------------------------------------------------------------
#endif  // C_ENABLE_CUSTOM_DEVICE_SUPPORT
//------------------------------------------------------------------------------
