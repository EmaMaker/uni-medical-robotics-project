# Description

This repo contains files and instructions for the project for the Medical Robotics exam at Sapienza University of Rome (AIRO/MBIR/MCER - Group 9)

The objective of the project is to execute a grasping task in a VR environment in [Unity game engine](https://unity3d.com) using the [WEART TouchDIVER G1 haptic interface](https://weart.it/haptic-vr-products/touchdiver-haptic-glove/).

In particular, the vanilla WEART SDK animates the hand interpolating between a _open_, _abducted_ and _closed_ animation, depending on a _closure_ parameter between [0, 1]. Each finger has its own _closure_ parameter, where 0 corresponds to a completely extended finger and 1 completely curled finger. Additionally, the thumb has an _abduction_ parameter, although its value is handled as another _closure_ in the communication between Unity and the [middleware](#middleware).

Our objective is instead to consider each finger as it were a robotic manipulator (e.g. a 3R planar robot for the index and middle fingers) and animate it using inverse kinematics. At the moment, this is achieved by tracking an arc of circle trajectory between the extended and curled position of the finger, using velocity control. Furthermore, a null space term is used to keep the joint angles in [0, PI/2], so to ensure a natural movement for the fingers.

In a second moment, the hand will be tracked with the inbuilt cameras of the VR headset instead of the haptic interface.

## WEART Middleware and communication with Unity

WEART offers a SDK for unity, c#, c++ and python.

In all cases, the SDK connects to a middleware, implemented only for Windows, which in turn communicates with the actual haptic interface. For the purpose of testing outside the university lab, we implemented a [fake middleware](./fakemiddleware): a python script that acts like the middleware, sending fake data to unity relative to interface status and closure values.
More info is in the dedicated folder.

- SDK Download: [https://weart.it/developer-guide-td-g1/](https://weart.it/developer-guide-td-g1/)
- SDK Reference: [https://weart.it/docs/sdkunity/2.0.0/](https://weart.it/docs/sdkunity/2.0.0/)

This project uses (and modifies) version 2.0.0 of the SDK. Install the version in this repo. To install it in Unity _Window>Package Manager>Plus icon>Install from disk_ and select the `package.json` in there.

The WEART documentation reports support for Unity 2021 and 2022. The 2022 version is LTS.

## Unity and Linear Algebra

Math.NET Numerics is used for Linear Algebra library in this project.
To install it in Unity, first install the [NuGetForUnity](https://github.com/GlitchEnzo/NuGetForUnity) plugin, which installs the NuGet Package Manager.
Download the [**.unitypackage** from releases](https://github.com/GlitchEnzo/NuGetForUnity/releases/tag/v4.5.0) and then in Unity [follow this guide](https://docs.unity3d.com/6000.0/Documentation/Manual/AssetPackagesImport.html) to install it.
Then restart Unity and in the toolbar a new submenu named NuGet will appear. Go to _Nuget>Manager NuGet Packages_ and install _Math.NET Numerics_

- Math.NET Numerics Documentation: [https://numerics.mathdotnet.com/api/Numerics.LinearAlgebra](https://numerics.mathdotnet.com/api/Numerics.LinearAlgebra)

## Folders

- `fakemiddleware` contains the [Fake Middleware](./fakemiddleware)
- `unity-project` contains the Unity Project, with a sample scene containing just the WEART Prefab, with the hands and calibration and info/stats panels
- `matlab-simulation` contains some matlab simulations, then reimplemented in unity
- `videos` contains some videos of the progress of the project. The name keeps the order in which they were taken and a brief description
- `WEART_SDK_Unity_v2.0.0` contains the modified version of the WEART SDK

# To Do

- Unity
    - [x] Animation in Unity using IK instead of interpolation of Animation States
        - [x] Index
        - [x] Middle + Other fingers
        - [x] Thumb
    - [ ] Grasping

- Camera
    - [ ] Machine vision
    - [ ] Animation using IK
    - [ ] Grasping
