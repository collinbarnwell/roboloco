#ifndef DEM_DEFS
#define DEM_DEFS


// **** OThER
#define INF 16384
#define PI 3.1415926

// **** FSPF

#define MAX_HOODS 1000 // kmax 3000
#define MAX_PTS 10000 // nmax 20000
#define LOCAL_SAMPS 80 // (l) num local samples
// #define PLANE_SIZE .5 // S
#define PLANE_OFFSET .03 // e
#define MIN_INLIER .9 // alpha-in
#define NEIGH_SIZE .25 // replaces Global_neigh
#define NEIGH_DENSITY 20
// #define GLOBAL_NEIGH 60 // n Neighborhood for global samples (in pixels) *** NOT USED ** 

// **** CGR 

// Map boundaries
#define MAPMAXY 7.62
#define MAPMAXX 10.21

// Max random variance to introduce when creating new particles based off old ones
#define ANGLE_VARIANCE 0.17 // in rads = ~10 degrees
#define DIST_VARIANCE 0.3 // in meters

#define PARTICLE_NUM 200

// Maximum error in normal angles to include a point
#define MAX_NORMAL_DIFF 0.0872 // 0.0872 = 5 degrees

// **** CGR Constants

// #define SIGMA .02 // standard deviation of distance measurement
// #define DISCOUNT 10 // discounting factor f <- ??????????
#define KONSTANT .08 // 2 * SIGMA^2 * f

// Keep best KEEP_RATIO percentage of particles
#define KEEP_RATIO .1 

// Create NEW_SAMPS new samples based off of eac kept particle
#define NEW_SAMPS 6


#endif
