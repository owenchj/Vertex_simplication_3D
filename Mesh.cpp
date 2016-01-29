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


#include "Mesh.h"
#include <iostream>
#include <fstream>
#include <cstdlib>

using namespace std;

void Mesh::loadOFF (const std::string & filename) {
	ifstream in (filename.c_str ());
    if (!in) 
        exit (1);
	string offString;
    unsigned int sizeV, sizeT, tmp;
    in >> offString >> sizeV >> sizeT >> tmp;
    V.resize (sizeV);
    T.resize (sizeT);
    for (unsigned int i = 0; i < sizeV; i++)
        in >> V[i].p;
    int s;
    for (unsigned int i = 0; i < sizeT; i++) {
        in >> s;
        for (unsigned int j = 0; j < 3; j++)
            in >> T[i].v[j];
    }
    in.close ();
    centerAndScaleToUnit ();
    recomputeNormals ();

    // Test function
    findNeighbours();
    shapeApproximation();
    
    // Vec3f a(0.0,3.0,0.0);
    // Vec3f b(4.0,0.0,0.0);
    // Vec3f c(0.0,0.0,0.0);
    // Proxy p(a,b);
    // distortionError(p,a,b,c);
}

void Mesh::recomputeNormals () {
    for (unsigned int i = 0; i < V.size (); i++)
        V[i].n = Vec3f (0.0, 0.0, 0.0);
    for (unsigned int i = 0; i < T.size (); i++) {
        Vec3f e01 = V[T[i].v[1]].p -  V[T[i].v[0]].p;
        Vec3f e02 = V[T[i].v[2]].p -  V[T[i].v[0]].p;
        Vec3f n = cross (e01, e02);
        n.normalize ();
        for (unsigned int j = 0; j < 3; j++)
            V[T[i].v[j]].n += n;
    }
    for (unsigned int i = 0; i < V.size (); i++)
        V[i].n.normalize ();
}

void Mesh::centerAndScaleToUnit () {
    Vec3f c;
    for  (unsigned int i = 0; i < V.size (); i++)
        c += V[i].p;
    c /= V.size ();
    float maxD = dist (V[0].p, c);
    for (unsigned int i = 0; i < V.size (); i++){
        float m = dist (V[i].p, c);
        if (m > maxD)
            maxD = m;
    }
    for  (unsigned int i = 0; i < V.size (); i++)
        V[i].p = (V[i].p - c) / maxD;
}





/// E = |ni - Ni|^2 * |Ti|; |Ti| = area of triangle T
float Mesh::distortionError(Proxy p, Vec3f t0, Vec3f t1, Vec3f t2) {
  Vec3f n;
  float err;
  float distance2, area;

  Vec3f e0 = t1 - t0;
  Vec3f e1 = t2 - t0;
  
  n = cross(e0, e1);
  area = 0.5 * n.length();

  n.normalize();
  distance2 = pow(dist(n, p.n), 2);
    
  err = distance2 * area;

  // cout <<n << endl;  
  // cout <<area<< endl;  
  // cout <<err<< endl;  
  
  return err;
}



bool Mesh::isNeighbour(Triangle &T0, Triangle &T1) {
  Vec3f t0[3];
  Vec3f t1[3];
  int cors = 0;

  for (unsigned int i = 0; i < 3; i++)
    {
      t0[i]= V[T0.v[i]].p ;
      t1[i]= V[T1.v[i]].p ;
    }

  for (unsigned int i = 0; i < 3; i++)
    {
      if(t0[i] == t1[0] || t0[i] == t1[1] || t0[i] == t1[2])
	cors++;
    }
  
  if(cors == 2)
    return true;
  else 
    return false;
}

void Mesh::shapeApproximation() {

  std::vector<Triangle > errQue(2 * T.size());
  
  int t[num] ;
  Proxy p[num];
  
  /* initialize random seed: */
  srand (time(NULL));
  
  /* generate secret number between 1 and 10: */
  for (unsigned int i = 0; i < num; i++){
    t[i]= rand() % T.size() + 1;
    cout << t[i] << endl;
  }
  
  for (unsigned int i = 0; i < num; i++)
    {

    }
  
  
}


void Mesh::findNeighbours() {

  int cnt = 0;
  Neighbours nb;
  
  for (unsigned int i = 0; i < T.size (); i++)
    {
      nb.t[0] = -1;
      nb.t[1] = -1;
      nb.t[2] = -1;
      cnt = 0;
      for (unsigned int j = 0; j < T.size (); j++){
	if(j != i){
	  if(isNeighbour(T[i], T[j]) == true){
	    nb.t[cnt++] = j;
	  }
	}
      }

      T[i].neighbour = nb;      
    }

   // for (unsigned int i = 0; i < T.size (); i++)
   //   {
   //     cout << T[i].neighbour.t[0] << " " ; 
   //     cout << T[i].neighbour.t[1] << " "; 
   //     cout << T[i].neighbour.t[2] << endl; 
   //   } 

   //recomputeNormals ();
}
