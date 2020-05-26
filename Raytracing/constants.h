#ifndef __CONSTANTS_H__
#define __CONSTANTS_H__

#pragma once
//Number of bounces of secondary rays
#define MAX_DEPTH 20

//Shadow type (true -> Soft Shadows, false->hard shadows)
#define SOFT_SHADOWS false

//Sample per Pixel (in truth this is the sqrt spp) [also number of rays to shoot in no antialiasing soft shadows]
#define SPP 20

//size of the side of the light jitter
#define LIGHT_SIDE .5f

//Hard colors for intersections test
#define TEST_INTERSECT false

//Sample unit disk? (false for normal jitter)
#define SAMPLE_DISK true

//Antialiasing flag (also turns on the DOF)
#define ANTIALIASING true

//Depth of field flag (for DOF to work, antialiasing must be true as well)
#define DEPTH_OF_FIELD true

//background color (true -> skybox; false -> background_color)
#define SKYBOX true

//test depth of intersections
#define DEPTH_MAP false

// use path tracing instead of ray tracing (not reccomended without antialiasing)
#define PATHTRACING true

#define GAMMA 1.0f


enum accel_struct {None, UGrid, Bvh};
enum sample_mode {jitter, tent};

accel_struct acl_str = accel_struct::Bvh;
sample_mode s_mode = sample_mode::jitter;

#endif // __CONSTANTS_H__