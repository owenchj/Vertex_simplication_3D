// --------------------------------------------------------------------------
// Copyright(C) 2009-2015
// Tamy Boubekeur
//                                                                            
// All rights reserved.                                                       
//                                                                            
// This program is free software; you can redistribute it and/or modify       
// it under the terms of the GNU General Public License as published by       
// the Free Software Foundation; either version 2 of the License, or          
// (at your option) any later version.                                        
//                                                                            
// This program is distributed in the hope that it will be useful,            
// but WITHOUT ANY WARRANTY; without even the implied warranty of             
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              
// GNU General Public License (http://www.gnu.org/licenses/gpl.txt)           
// for more details.                                                          
// --------------------------------------------------------------------------

#pragma once
#include <cmath>
#include <vector>
#include "Vec3.h"
#include <time.h>       /* time */

#define num 6


///
class Neighbours {
 public:
  inline Neighbours () { t[0] = 0; t[1] = 0; t[2] =  0; }
  inline Neighbours (const int & t0, const int & t1, const int & t2) {
    t[0] = t0; t[1] = t1; t[2] =  t2;
  }
  inline virtual ~Neighbours () {}
  int t[3];
};

/// A simple vertex class storing position and normal
class Vertex {
 public:
  inline Vertex () {}
  inline Vertex (const Vec3f & p, const Vec3f & n) : p (p), n (n) {}
  inline virtual ~Vertex () {}
  Vec3f p;
  Vec3f n;
};

/// A Triangle class expressed as a triplet of indices (over an external vertex list)
class Triangle {
 public:
  inline Triangle () {
    v[0] = v[1] = v[2] = 0;
    lable = -1;
    tag = -1;
  }
  inline Triangle (const Triangle & t) {
    v[0] = t.v[0];
    v[1] = t.v[1];
    v[2] = t.v[2];
    lable = t.lable;
    tag = t.tag;
  }
  inline Triangle (unsigned int v0, unsigned int v1, unsigned int v2) {
    v[0] = v0;
    v[1] = v1;
    v[2] = v2;
    lable = -1;
    tag = -1;
  }
  inline virtual ~Triangle () {}
  inline Triangle & operator= (const Triangle & t) {
    v[0] = t.v[0];
    v[1] = t.v[1];
    v[2] = t.v[2];
    lable = t.lable;
    tag = t.tag;
    return (*this);
  }
  unsigned int v[3];
  Neighbours neighbour;
  int lable;
  int tag;
};

class Proxy {
 public:
  inline Proxy () {}
  inline Proxy (const Vec3f & x, const Vec3f & n) : x (x), n (n) {}
  inline virtual ~Proxy () {}
  Vec3f x;
  Vec3f n;
  // TODO
  std::vector<Triangle> T;
};

/// A Mesh class, storing a list of vertices and a list of triangles indexed over it.
class Mesh {
 public:
  std::vector<Vertex> V;
  std::vector<Triangle> T;

  /// Loads the mesh from a <file>.off
  void loadOFF (const std::string & filename);
    
  /// Compute smooth per-vertex normals
  void recomputeNormals ();

  /// scale to the unit cube and center at original
  void centerAndScaleToUnit ();

  /// distortion error
  float distortionError(Proxy p, Vec3f t0, Vec3f t1, Vec3f t2);
    
  /// shape approximation
  void shapeApproximation() ;

  /// shape approximation
  bool isNeighbour(Triangle &T0, Triangle &T1);

  void findNeighbours();

};
