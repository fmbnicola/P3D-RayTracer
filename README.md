# Whitted Ray Tracer + Simple Unbiased Path Tracer
This project was developed in the context of the 3D Programming course at Instituto Superior Técnico.

<p align="center">
  <img src="https://user-images.githubusercontent.com/25552993/114410920-a7ac6600-9ba3-11eb-9ad8-29f9f1011402.png" />
</p>

It features a Whitted Ray Tracer capable of rendering simple scenes with triangles, spheres, planes, and aabb. It comes with some simple sample scenes in a custom file format ".p3f" (stemming from ".nft" format). The Raytracer uses the Blinn-Phong Reflection model to shade diffuse, refractive, and reflective materials. It also supports the use of a sky-box and a Depth of field effect.

The Unbiased Path Tracer is based on the smallpt project by Kevin Beason: http://www.kevinbeason.com/smallpt/ 
It produces images using global illumination and emissive materials as light sources. 

A Uniform Grid and a Bounding Volume Hierarchy were implemented for both renderers as acceleration data structures. 

**Ray tracer demo:**
https://vimeo.com/411861765

**Path Tracer demo:**
https://vimeo.com/424598721

**Created by:**
Francisco Nicolau, Francisco Sousa and João Martinho
