#ifndef CITY_MODELS_H
#define CITY_MODELS_H

#include "geometry3d.h"

/*
 * Unit rectangular plane in the XZ plane.
 * Its perimeter edges are rendered; its triangles supply depth.
 */
extern const geometry3d_model_t geometry3d_plane_model;

/* Unit plus-shaped street surface in the XZ plane. */
extern const geometry3d_model_t geometry3d_plus_road_model;

#endif /* CITY_MODELS_H */
