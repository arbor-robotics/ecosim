---
title: The Warthog
layout: default
---

Here is how the Warthog is modeled:

1. It's modeled in Blender, which creates a `.blend` file. This file is the source of all other files, and is therefore the most important one.

2. It's exported as a `.fbx` file and added to Unity.

3. Unity treats `.fbx` files as prefabs. You may drag it directly into a scene. If the `.fbx` file is updated (if you re-export from Blender), the pre-fabs in the Unity scene will automatically be updated.