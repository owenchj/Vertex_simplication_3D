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

/// proxy numbers
#define num 20
/// iteration num 
#define iteNum 10
/// threshold for edge extraction
#define threshold 0.25
/// open debug cout
#define debug 0





/// a simple vertex class storing position and normal
class Vertex {
 public:
  inline Vertex () {lable = -1; index = -1;}
  inline Vertex (const Vec3f & p, const Vec3f & n, int & lable, int & index) : p (p), n (n), lable(lable), index(index){}
  inline virtual ~Vertex () {}
  Vec3f p;
  Vec3f n;
  std::vector<int > proxies;  
  int lable;
  int index;
};

/// A Triangle class expressed as a triplet of indices (over an external vertex list)
class Triangle {
 public:
  inline Triangle () {
    v[0] = v[1] = v[2] = 0;
    lable = -1;
    tag = -1;
    err = -1.0;
    index = -1;
  }
  inline Triangle (const Triangle & t) {
    v[0] = t.v[0];
    v[1] = t.v[1];
    v[2] = t.v[2];
    neighbours = t.neighbours;
    lable = t.lable;
    tag = t.tag;
    err = t.err;
    n = t.n;
    index = t.index;
  }
  inline Triangle (unsigned int v0, unsigned int v1, unsigned int v2) {
    v[0] = v0;
    v[1] = v1;
    v[2] = v2;
    lable = -1;
    tag = -1;
    err = -1.0;
    index = -1;
  }
  inline virtual ~Triangle () {}
  inline Triangle & operator= (const Triangle & t) {
    v[0] = t.v[0];
    v[1] = t.v[1];
    v[2] = t.v[2];
    neighbours = t.neighbours;
    lable = t.lable;
    tag = t.tag;
    err = t.err;
    n = t.n;
    index = t.index;
    return (*this);
  }
  unsigned int v[3];
  std::vector<int > neighbours;
  int lable;
  int tag;
  float err;
  Vec3f n;
  int index;

};


class anchorPair {
 public:
  inline anchorPair () {}
  inline anchorPair (const Vertex & v0, const Vertex & v1, int p0, int p1) {
    anchorVertex[0] = v0;
    anchorVertex[1] = v1;
    proxy[0] = p0;
    proxy[1] = p1;
  }
  
  inline virtual ~anchorPair () {}
  Vertex anchorVertex[2];
  int proxy[2];
  std::vector<Vertex > edges;  
};



class Proxy {
 public:
  inline Proxy () { added = -1;}
  inline Proxy (const Vec3f & x, const Vec3f & n, int added) : x (x), n (n), added(added) {}
  inline virtual ~Proxy () {}
  Vec3f x;
  Vec3f n;
  // TODO
  std::vector<Vertex > V;  
  std::vector<Triangle> T;
  std::vector<int> adjacentProxy;
  std::vector<Vertex > anchorV;  

  int added;
};

/// A Mesh class, storing a list of vertices and a list of triangles indexed over it.
class Mesh {
 public:

  Proxy p[num];
  int seed[num];  
  bool flagFirst;
  std::vector<Vertex> V;
  std::vector<Triangle> T;
  std::vector<Triangle> errQue;

  std::vector<Vertex> VR;
  std::vector<Triangle> TR;

  std::vector<Vertex> anchorV;
  std::vector<Vertex> edgeV;
  std::vector<anchorPair> anchorP;

  inline Mesh () {
    flagFirst = true; 
    for (unsigned int i = 0; i < num; i++)  seed[i] = -1;
  }
    
  /// Loads the mesh from a <file>.off
  void loadOFF (const std::string & filename);
    
  /// Compute smooth per-vertex normals
  void recomputeNormals ();

  /// scale to the unit cube and center at original
  void centerAndScaleToUnit ();

  /// distortion error
  float distortionError(Proxy &p, Triangle &T);
    
  /// shape approximation =>  partitioning
  void partitioning() ;

  void proxyFitting() ;
  
  
  void calculateNormal(Triangle &T) ;
  
  /// shape approximation
  bool isNeighbour(Triangle &T0, Triangle &T1);
  
  void findNeighbours();

  //  Triangle popLeastErrTriangle(std::vector<Triangle > &errQue);
  Triangle popLeastErrTriangle();

  // avoid repeated random num
  bool repeatedNum(int i);
  
  void remesh();

  // avoid add repeated triangle to result
  bool isSame(Triangle &T0, Triangle &T1);
  
  bool isLine(Vertex & V0, Vertex & V1, int *p);
  
  bool isinLine(Vertex & V, anchorPair & P);
  
  void vertexClass();

  void edgeExtraction();

  void findMoreAnchor();

  float distanceToLine(Vertex & V, anchorPair & P);

  void triangulation();
};
