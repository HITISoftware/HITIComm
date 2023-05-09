/*
 * HITIComm
 * HC_ServoRobot.h
 *
 * Copyright © 2021 Christophe LANDRET
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *         http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */



// *****************************************************************************
// Include Guard
// *****************************************************************************

#ifndef HC_ServoRobot_h
#define HC_ServoRobot_h



// *****************************************************************************
// Include dependencies
// *****************************************************************************

// HITICommSupport
#include <HITICommSupport.h>



// *****************************************************************************
// Forward declaration of class
// *****************************************************************************

class Servo;	     // because used as a pointer
class HC_Servo;	     // because used as a pointer (to an array)
class HC_MotorGroup; // because used as a pointer (to an array)



// *****************************************************************************
// Define
// *****************************************************************************



// *****************************************************************************
// Class
// *****************************************************************************

class HC_ServoRobot
{
	public:
		// constructor *****************************************************
		HC_ServoRobot();
		
		HC_ServoRobot(
				uint8_t ID,
				uint8_t motor_qty,
				uint8_t motorGroup_qty);
		
		// destructor ******************************************************
		~HC_ServoRobot();
		
		// getters *********************************************************
		
		
		// setters *********************************************************
		void init(
				uint8_t ID,
				uint8_t motor_qty,
				uint8_t motorGroup_qty);
		
		void initServo(
				uint8_t motorIndex, 
				uint8_t pin,
				bool invertedDirection, 
				int offset,
				float initPosition,
				Servo* servo = NULL_POINTER);
		void initServo(
				uint8_t motorIndex, 
				uint8_t pin,
				bool invertedDirection, 
				int offset,
				int minPulseWidth,
				int maxPulseWidth,
				float initPosition,
				Servo* servo = NULL_POINTER);
				
		void initMotorGroup(
				uint8_t groupIndex,
				uint8_t motorQty);	
	
		// position and speed modes
		/*void posMode_absolute();
		void posMode_relative();
		void motionTime(float motionTime);*/

		/*void setMotionMode(
				HC_PositionMode positionMode,
				HC_SpeedMode speedMode);
		void setMotionMode(
				uint8_t motorIndex,
				HC_PositionMode positionMode,
				HC_SpeedMode speedMode);*/
		
		// position and speed setpoints (according to selected modes)			
		/*void setMotionSetpoint(
				uint8_t motorIndex,
				float posSetpoint,	
				float speSetpoint);			*/
				
		void reset(uint8_t groupIndex);		
				
		// getters *********************************************************
		HC_Servo& getServo(uint8_t motorIndex) const;
						
		bool isReady() const;
		bool isMoving() const;
		bool isDone() const;
		
		bool isReady(uint8_t groupIndex) const;
		bool isMoving(uint8_t groupIndex) const;
		bool isDone(uint8_t groupIndex) const;	
						
		// group management ************************************************
		void addMotorToGroup(
				uint8_t groupIndex,
				uint8_t motorIndex);	
				
		// motion **********************************************************
		void moveOnTrigger(
				bool trigger,
				uint8_t groupIndex);			
		void moveNow(uint8_t groupIndex);	
		void stopOnTrigger(
				bool trigger,
				uint8_t groupIndex);		
		void stopNow(uint8_t groupIndex);
		
	protected:
	
	private:
		// methods *********************************************************
		
		// arrays management
		void clear();


		// variables *******************************************************
		uint8_t mID;							// robot ID
		uint8_t mMotor_qty;					
		HC_Servo* mMotor_array;				// array of servos
		uint8_t mGroup_qty;	
		HC_MotorGroup* mGroup_array;	// array of Motor Group
};


#endif
