---
title: Home
layout: home
nav_order: 1
---

<iframe width="560" height="315" src="https://www.youtube-nocookie.com/embed/MP3kJ36SKxM?si=eMX4H46jnp1a72wU" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

# About EcoSim

EcoSim is a custom simulator for the Digital Steward, built in Unity.

# Building from source

### 1. Add the project to Unity

In the Unity Hub, navigate to Projects. Click the arrow next to the Add button and select "Add project from disk." Select EcoSim's root folder.

You may be prompted to install the appropriate editor version for the project. At the time of writing, this is "2022.3.17f1."

### 2. Install dependencies

The first time you open the project, Unity will go through a long compilation step. Once the editor opens, you may notice a number of errors due to missing packages.

To fix this, navigate to Window > Package Manager. If this is your first time installing this project with your Unityaccount, you will need to add dependencies through the Unity Asset Store on the web by navigating to these links and clicking "Add to My Assets," followed by "Open in Unity":

- [Unity Terrain - HDRP Demo Scene](https://assetstore.unity.com/packages/3d/environments/unity-terrain-hdrp-demo-scene-213198)

Back in the project manager, for each dependency, click "Download" and "Import." Importing packages can take some time, so be patient. It only needs to be done once.

{: .highlight }
If you do not see your assets, make sure you've selected "Packages: My Assets" in the dropdown in the top left of the Package Manager.