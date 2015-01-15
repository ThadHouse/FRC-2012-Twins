/////////////////////////////////////////////////////////////////////////
//
// This module contains the Gesture Processing code for the 
// FRC 2012 Kinect Server
//
// Copyright (c) FIRST 2011. All Rights Reserved.							  
// Open Source Software - may be modified and shared by FRC teams. The code   
// must be accompanied by, and comply with the terms of, the license found at
// \FRC Kinect Server\License_for_KinectServer_code.txt which complies
// with the Microsoft Kinect for Windows SDK (Beta) 
// License Agreement: http://kinectforwindows.org/download/EULA.htm
//
/////////////////////////////////////////////////////////////////////////

using System;
using System.Collections.Generic;
using System.Text;
using System.Linq;
using Microsoft.Research.Kinect.Nui;
using Edu.FIRST.WPI.Kinect.KinectServer.Networking.WritableElements;

namespace Edu.FIRST.WPI.Kinect.KinectServer.Kinect
{
    /// <summary>
    /// Process the FIRST standard gestures:
    ///  - Left arm inclination from 0: Joystick 0 Y axis, from -128 to 127
    ///  - Right arm inclination from 0: Joystick 1 Y axis, from -128 to 127
    ///  
    ///  - Buttons:
    ///     - Head:
    ///         - Tilted to the right: Btn0
    ///         - Tilted to the left: Btn1        
    ///     - Right leg:
    ///         - Out (extended to the right): Btn2
    ///         - Forward: Btn4
    ///         - Backward: Btn5
    ///     - Left leg:
    ///         - Out (extended to the left): Btn3
    ///         - Forward: Btn6
    ///         - Backward: Btn7
    ///         
    /// Button values the same for both joysticks.
    /// </summary>
    class FIRSTGestureProcessor: IGestureProcessor
    {
        public const double Z_PLANE_TOLERANCE = 0.3;

        public const double ARM_MAX_ANGLE = 105;
        public const double ARM_MIN_ANGLE = -90;

        delegate bool CheckAngle(double angle);

        CheckAngle IsLegForward = x => x < -110;
        CheckAngle IsLegBackward = x => x > -80;
        CheckAngle IsLegOut = x => x > -75;
        CheckAngle IsHeadLeft = x => x > 98;
        CheckAngle IsHeadRight = x => x < 82;

        DateTime lastAdjustment = DateTime.Now;

        /// <summary>
        /// Processes a skeleton into joystick data using the default FIRST gestures.
        /// </summary>
        /// <param name="joy">Vector of Joysticks to put the result in</param>
        /// <param name="skeleton">The skeleton to process</param>
        public void ProcessGestures(Networking.WritableElements.WritableJoystick[] joy, Microsoft.Research.Kinect.Nui.SkeletonData skeleton, Microsoft.Research.Kinect.Nui.Camera cam)
        {
            
            var cur = DateTime.Now;
            if (cur.Subtract(lastAdjustment) > TimeSpan.FromSeconds(1))
            {
                //5 degrees = .1 error
                double headPosition = skeleton.Joints[JointID.Head].Position.Y;
                int currentAngle = cam.ElevationAngle;
                int newAngle = currentAngle;
                double headError = headPosition - 0.4;

                double adjust = headError * 50;

                if (adjust > 0 && adjust < 1.51)
                {
                    adjust = 0;
                }
                else if (adjust < 0 && adjust > -1.51)
                {
                    adjust = 0;
                }
                newAngle = currentAngle + (int)adjust;

                if (newAngle == Camera.ElevationMaximum)
                {
                    newAngle = Camera.ElevationMaximum;
                }
                if (newAngle <= 0)
                {
                    newAngle = 0;
                }
                lastAdjustment = cur;
                if (newAngle != currentAngle)
                {
                    try
                    {
                        cam.ElevationAngle = newAngle;
                    }
                    catch
                    {
                    }
                }
            }


            // Check edge cases
            if (joy == null || joy.Length < 2 || joy[0] == null || joy[1] == null)
                return;
            
            sbyte[] leftAxis = new sbyte[6];
            sbyte[] rightAxis = new sbyte[6];
            sbyte[] nullAxis = new sbyte[6];
            bool dataWithinExpectedRange;
            ushort buttons = 0;

            double cameraAngle = cam.ElevationAngle;


            double leftAngle = RadToDeg(AngleXY(skeleton.Joints[JointID.ShoulderLeft].Position,
                                                skeleton.Joints[JointID.WristLeft].Position,
                                                true));

            double rightAngle = RadToDeg(AngleXY(skeleton.Joints[JointID.ShoulderRight].Position,
                                                 skeleton.Joints[JointID.WristRight].Position));

            dataWithinExpectedRange = leftAngle < ARM_MAX_ANGLE && leftAngle > ARM_MIN_ANGLE &&
                                      rightAngle < ARM_MAX_ANGLE && rightAngle > ARM_MIN_ANGLE;

            double leftYAxis = CoerceToRange(leftAngle,
                                             -70,
                                             70,
                                             -127,
                                             128);

            double rightYAxis = CoerceToRange(rightAngle,
                                             -70,
                                             70,
                                             -127,
                                             128);

            dataWithinExpectedRange = dataWithinExpectedRange &&
                                      InSameZPlane(skeleton.Joints[JointID.ShoulderLeft].Position,
                                                   skeleton.Joints[JointID.WristLeft].Position,
                                                   Z_PLANE_TOLERANCE) &&
                                      InSameZPlane(skeleton.Joints[JointID.ShoulderRight].Position,
                                                   skeleton.Joints[JointID.WristRight].Position,
                                                   Z_PLANE_TOLERANCE);


            // Head buttons
            double headAngle = RadToDeg(AngleXY(skeleton.Joints[JointID.ShoulderCenter].Position,
                                                skeleton.Joints[JointID.Head].Position));
            if (IsHeadRight(headAngle))
                buttons |= (ushort) WritableJoystick.Buttons.Btn1;
            if (IsHeadLeft(headAngle))
                buttons |= (ushort)WritableJoystick.Buttons.Btn2;


            // Right Leg XY Button
            double rightLegAngle = RadToDeg(AngleXY(skeleton.Joints[JointID.HipRight].Position,
                                                    skeleton.Joints[JointID.AnkleRight].Position));
            if (IsLegOut(rightLegAngle))
                buttons |= (ushort)WritableJoystick.Buttons.Btn3;

            // Left Leg XY Button
            double leftLegAngle = RadToDeg(AngleXY(skeleton.Joints[JointID.HipLeft].Position,
                                                   skeleton.Joints[JointID.AnkleLeft].Position,
                                                   true));
            if (IsLegOut(leftLegAngle))
                buttons |= (ushort)WritableJoystick.Buttons.Btn4;

            // Right Leg YZ Buttons
            double rightLegYZ = RadToDeg(AngleYZ(skeleton.Joints[JointID.HipRight].Position,
                                                 skeleton.Joints[JointID.AnkleRight].Position));
            if (IsLegForward(rightLegYZ))
                buttons |= (ushort)WritableJoystick.Buttons.Btn5;
            if (IsLegBackward(rightLegYZ))
                buttons |= (ushort)WritableJoystick.Buttons.Btn6;

            // Left Leg YZ Buttons
            double leftLegYZ = RadToDeg(AngleYZ(skeleton.Joints[JointID.HipLeft].Position,
                                                skeleton.Joints[JointID.AnkleLeft].Position));
            if (IsLegForward(leftLegYZ))
                buttons |= (ushort)WritableJoystick.Buttons.Btn7;
            if (IsLegBackward(leftLegYZ))
                buttons |= (ushort)WritableJoystick.Buttons.Btn8;

            double distance = skeleton.Joints[JointID.Spine].Position.Z;
            double distanceFT = distance * 3.2808399;
            distance = CoerceToRange(distance, 0, 5, -127, 128);
            
            if (distanceFT > 8)
                buttons |= (ushort)WritableJoystick.Buttons.Btn12;

            double libraryYAxis = CoerceToRange(leftAngle, -70, 70, -1, 1);
            double libraryXAxis = CoerceToRange(rightAngle, -70, 70, -1, 1);

            if (libraryXAxis > 0.6 || libraryYAxis > 0.6)
                buttons |= (ushort)WritableJoystick.Buttons.Btn11;

            if (dataWithinExpectedRange)
            {
                // Invert joystick axis to match a real joystick
                // (pushing away creates negative values)
                leftAxis[(uint)WritableJoystick.Axis.Y] = (sbyte) -leftYAxis;
                rightAxis[(uint)WritableJoystick.Axis.Y] = (sbyte) -rightYAxis;
                rightAxis[(uint)WritableJoystick.Axis.Custom] = (sbyte)distance;
                leftAxis[(uint)WritableJoystick.Axis.Custom] = (sbyte)distance;
                leftAxis[(uint)WritableJoystick.Axis.Throttle] = (sbyte)cameraAngle;

                //Use "Button 9" as Kinect control enabled signal
                buttons |= (ushort)WritableJoystick.Buttons.Btn9;

                joy[0].Set(leftAxis, buttons);
                joy[1].Set(rightAxis, buttons);
            }
            else
            {
                nullAxis[(uint)WritableJoystick.Axis.Custom] = (sbyte)distance;
                nullAxis[(uint)WritableJoystick.Axis.Throttle] = (sbyte)cameraAngle;
                joy[0].Set(nullAxis, buttons);
                joy[1].Set(nullAxis, buttons);
                
            }
        }

        /// <summary>
        /// Converts units from radians to degrees.
        /// </summary>
        /// <param name="rad">A value in radians.</param>
        /// <returns>The given value in degrees.</returns>
        private double RadToDeg(double rad)
        {
            return (rad * 180 ) / Math.PI;
        }

        /// <summary>
        /// Calculates the XY plane tangent between the given vectors.
        /// </summary>
        /// <param name="origin">The first point.</param>
        /// <param name="measured">The second point.</param>
        /// <param name="mirrored">Whether or not to invert the X axis.</param>
        /// <returns></returns>
        private double AngleXY(Vector origin, Vector measured, bool mirrored = false)
        {
            return Math.Atan2(measured.Y - origin.Y, (mirrored) ? (origin.X - measured.X) : (measured.X - origin.X));
        }

        /// <summary>
        /// Calculates the YZ plane tangent between the given vectors.
        /// </summary>
        /// <param name="origin">The first point.</param>
        /// <param name="measured">The second point.</param>
        /// <param name="mirrored">Whether or not to invert the Z axis.</param>
        /// <returns></returns>
        private double AngleYZ(Vector origin, Vector measured, bool mirrored = false)
        {
            return Math.Atan2(measured.Y - origin.Y, (mirrored) ? (origin.Z - measured.Z) : (measured.Z - origin.Z));
        }

        /// <summary>
        /// Determines whether the given points lie in the same XY plane along the Z axis.
        /// </summary>
        /// <param name="origin">The first point.</param>
        /// <param name="measured">The second point.</param>
        /// <param name="tolerance">The acceptable tolerance between the XY planes of the given points.</param>
        /// <returns>Whether or not the given points are close enough along the Z axis.</returns>
        private bool InSameZPlane(Vector origin, Vector measured, double tolerance)
        {
            return Math.Abs(measured.Z - origin.Z) < tolerance;
        }

        /// <summary>
        /// Converts an input value in the given input range into an output value along the given
        /// output range.
        /// 
        /// If the result would be outside of the given output range, it is constrained to the 
        /// output range.
        /// </summary>
        /// <param name="input">An input value within the given input range.</param>
        /// <param name="inputMin">The minimum expected input value.</param>
        /// <param name="inputMax">The maximum expected input value.</param>
        /// <param name="outputMin">The minimum expected output value.</param>
        /// <param name="outputMax">The maximum expected output value.</param>
        /// <returns>An output value within the given output range proportional to the input.</returns>
        private double CoerceToRange(double input, double inputMin, double inputMax, double outputMin, double outputMax)
        {
            // Determine the center of the input range
            double inputCenter = Math.Abs(inputMax - inputMin) / 2 + inputMin;
            double outputCenter = Math.Abs(outputMax - outputMin) / 2 + outputMin;

            // Scale the input range to the output range
            double scale = (outputMax - outputMin) / (inputMax - inputMin);

            // Apply the transformation
            double result = (input + -inputCenter) * scale + outputCenter;

            // Constrain to the result range
            return Math.Max(Math.Min(result, outputMax), outputMin);
        }
    }
}
