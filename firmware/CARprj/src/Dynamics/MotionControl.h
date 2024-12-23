#ifndef _MOTIONCONTROL_H_
#define _MOTIONCONTROL_H_

#include "FKIK.h"       // Forward and Inverse Kinematics library
#include <WiFiClient.h>
#include <stdint.h>     // For uint8_t, uint16_t

// Cycloid parameters
#define XS      90.0    // X origin   40 ~ 140             (50)
#define YS      -130.0  // Y origin   x = 90:-88 ~ -172    (42)
#define XMOVE   40      // X distance
#define H       20      // Lift height

class Robot
{
    public:
        bool RobotStatus = 1;      // 0->Track mode; 1->Quadruped mode

        void begin();       // Initialization

        uint8_t legNum = 1;             // Leg number for single-leg operations
        
        /* ------------------ Mode Control ------------------ */
        
        void FK_ResetQuadruped();       // Quadruped mode -- Forward Kinematics posture
        void IK_ResetQuadruped();       // Quadruped mode -- Inverse Kinematics posture

        void ResetTrack(bool status);   // Track mode -- Basic posture
        void FK_TStatus_HIGH();         // Track mode -- High posture
        void FK_TStatus_HIGHER();       // Track mode -- Highest posture

        void FK_LEFT();                 // Left tilt posture
        void FK_RIGHT();                // Right tilt posture
        void HoldInitialPosition();
        /* ------------------ Single Leg Control ------------------ */
        uint16_t AngleToCount(float angle);  // Convert angle to PCA9685 count

        // Forward Kinematics movements
        void FK_LUMove(float posk, float posx, uint16_t Time);    // Left front leg
        void FK_RUMove(float posk, float posx, uint16_t Time);    // Right front leg
        void FK_LBMove(float posk, float posx, uint16_t Time);    // Left back leg
        void FK_RBMove(float posk, float posx, uint16_t Time);    // Right back leg

        void FK_LegMove(float Angle[], uint8_t legNum, uint16_t DSD, bool Print);   // Single leg forward kinematics

        void FK_LSMove(float posk, uint16_t Time);               // Left side legs
        void FK_RSMove(float posk, uint16_t Time);               // Right side legs

        public:
        // Inverse Kinematics
            float IK_RUPoint[2] = {}; // Coordinate point storage
            float IK_LUPoint[2] = {};
            float IK_RBPoint[2] = {};
            float IK_LBPoint[2] = {};

        void IK_LUMove(float x, float y, uint16_t Time); // Left front leg
        void IK_RUMove(float x, float y, uint16_t Time); // Right front leg
        void IK_LBMove(float x, float y, uint16_t Time); // Left back leg
        void IK_RBMove(float x, float y, uint16_t Time); // Right back leg

        void IK_LegMove(float point[], uint8_t LEGNum, uint16_t DSD, bool Print); // Single leg inverse kinematics

        void LegPointDebug(float point[], uint8_t cmd, uint8_t offset, bool PrintPoint, bool PrintAngle); // Single leg debugging

        /* --- Cycloid --- */
        float F1_CPoints[20][2] = {}; // Forward cycloid 1
        float F2_CPoints[20][2] = {}; // Forward cycloid 2
        float F3_CPoints[20][2] = {}; // Forward cycloid 3

        float B1_CPoints[20][2] = {}; // Backward cycloid 1
        float B2_CPoints[20][2] = {}; // Backward cycloid 2
        float B3_CPoints[20][2] = {}; // Backward cycloid 3

        void GetCycloidPoints(float CPoints[][2], float xs, float xf, float ys, float yh); // Generate cycloid trajectory
        void LegCycloid(float CPoints[][2], uint8_t LEGNum);                                // Leg cycloid movement

        /* ------------------ Posture Control ------------------ */
        void PosMove(float xMove, float yMove, uint16_t Time); // Relative linear movement of posture
        void PosAction1();                                     // Posture action 1

        void PosToPoint(float x, float y, uint16_t Time); // Move to coordinate position
        void PosToPitch(float deg, uint16_t Time);        // Adjust pitch angle
        void PosToRoll(float deg, uint16_t Time);         // Adjust roll angle
        void PosAction2(uint8_t Num);                     // Posture action 2
        
        void PosAction3();                   // Posture action 3
        void PosAction4();                   // Posture action 4

        /* ------------------ Motion Control ------------------ */
        bool TrotStatus = 0;      // 0->Stop; 1->Trot
        bool WalkStatus = 0;      // 0->Stop; 1->Walk

        void Trot();        // Trot gait
        void Walk();        // Walk gait
        void Walk_Basic(uint8_t StepNum, bool dir); // Basic walking gait

        void VMC(float j0, float j1, float VMCPoint[], float VMCPoint_last[]); // VMC algorithm framework (testing)

        /* ------------------ Action Design ------------------ */
        void Hello(); // Wave hello
};

#endif
