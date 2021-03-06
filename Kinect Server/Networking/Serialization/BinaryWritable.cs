﻿/////////////////////////////////////////////////////////////////////////
// Copyright (c) FIRST 2011. All Rights Reserved.							  
// Open Source Software - may be modified and shared by FRC teams. The code   
// must be accompanied by, and comply with the terms of, the license found at
// \FRC Kinect Server\License_for_KinectServer_code.txt which complies
// with the Microsoft Kinect for Windows SDK (Beta) 
// License Agreement: http://kinectforwindows.org/download/EULA.htm
/////////////////////////////////////////////////////////////////////////


using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;

namespace Edu.FIRST.WPI.Kinect.KinectServer.Networking.Serialization
{
    /// <summary>
    /// Interface for objects which intend to serialize binary data
    /// for sending over the network.
    /// </summary>
    interface IBinaryWritable
    {
        /// <summary>
        /// When called, the associated object writes it's contained
        /// information to it's Stream reference.
        /// </summary>
        /// <param name="writer">The BinaryWriter to use for serialization.</param>
        void Serialize(NetworkOrderBinaryWriter writer);
    }
}
