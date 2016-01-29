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



/// E = ||ni - Ni||^2 * |Ti|; |Ti| = area of triangle T
float Mesh::distortionError(Proxy &p, Triangle & T) {
  Vec3f n;
  float err;
  float distance2, area;

  n = T.n;  
  area = 0.5 * n.length();
  distance2 = pow(dist(n, p.n), 2);

  err = distance2 * area;

  // cout <<n << endl;  
  // cout <<area<< endl;  
  // cout <<err<< endl;  
  
  return err;
}


void Mesh::calculateNormal(Triangle &T) {
  Vec3f t0 =  V[T.v[0]].p ;
  Vec3f t1 =  V[T.v[1]].p ;
  Vec3f t2 =  V[T.v[2]].p ;

  Vec3f e0 = t1 - t0;
  Vec3f e1 = t2 - t0;
  
  Vec3f n = cross(e0, e1);
  n.normalize();
  T.n = n;
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


void Mesh::findNeighbours() {

  for (unsigned int i = 0; i < T.size (); i++)
    {
      for (unsigned int j = 0; j < T.size (); j++){
	if(j != i && isNeighbour(T[i], T[j]) == true)
	  T[i].neighbours.push_back(j);
      }
      T[i].index = i;
      calculateNormal(T[i]);
    }

    // for (unsigned int j = 0; j < T.size (); j++)   
    //   cout << T[j].lable << endl;
  
#if debug == 1
  for (unsigned int i = 0; i < T.size (); i++)
    {
      for (std::vector<int>::iterator it = T[i].neighbours.begin() ; it != T[i].neighbours.end(); ++it)
  	std::cout << ' ' << *it;
      std::cout << '\n';
    } 
#endif
}


Triangle Mesh::popLeastErrTriangle(std::vector<Triangle > &errQue){
  Triangle t;
  int index = 0;
  float minerr = 1000.0;
  for (unsigned int i = 0; i < errQue.size (); i++){
    if(errQue[i].err < minerr){
      minerr = errQue[i].err;
      index = i;
    }
  }
  
  t = errQue[index];
  errQue.erase(errQue.begin()+index);
  
#if debug == 1
  for (std::vector<Triangle >::iterator it = errQue.begin() ; it != errQue.end(); ++it)
    std::cout << it->err << ' ';
#endif
  
  return t;
}

void Mesh::shapeApproximation() {

  std::vector<Triangle > errQue;
  //  Proxy p[num];
  int seed[num];  
  Triangle popTriangle;

  /* initialize random seed: */
  srand (time(NULL));
  
  /* generate secret number between 1 and 10: */
  for (unsigned int i = 0; i < num; i++){
    seed[i]= rand() % T.size() + 1;
    p[i].T.push_back(T[seed[i]]);
    p[i].x = (V[p[i].T[0].v[0]].p + V[p[i].T[0].v[1]].p + V[p[i].T[0].v[2]].p)/3;
    p[i].n = p[i].T[0].n;



#if debug == 1
    cout <<"Init Proxy Barycenter: " << p[i].x << endl;
    cout << "Init Proxy Normal: "<< p[i].n << endl;
    cout << seed[i] << endl;
#endif
  }
  
  // initialization for the error queque
  for (unsigned int i = 0; i < num; i++)
    {
#if debug == 1
      // see wheather initialization is sucessful
      cout << p[i].T.size() << endl;
      cout << p[i].T[0].lable<< endl;
      cout << p[i].T[0].tag<< endl;
      cout << p[i].T[0].err<< endl;
      cout << p[i].n << endl;
      cout << p[i].T[0].neighbours.size() << endl;    
#endif

      for (std::vector<int>::iterator it = p[i].T[0].neighbours.begin() ; it != p[i].T[0].neighbours.end(); ++it)
	{
	  T[*it].tag = i;
	  T[*it].err = distortionError(p[i], T[*it]);
	  // std::cout << ' ' << T[*it].err;
	  errQue.push_back(T[*it]);
	}
      std::cout << '\n';
    }
    
  //  std::cout <<"Initial Queque Size :" << errQue.size() << endl;

  while(!errQue.empty()){
    //std::cout <<"Initial Queque Size :" << errQue.size() << endl;
    /// pop the triangle with the smallest error
    popTriangle = popLeastErrTriangle(errQue);


    if ( T[popTriangle.index].lable == -1){
      T[popTriangle.index].lable = popTriangle.tag;
      p[popTriangle.tag].T.push_back(T[popTriangle.index]);

      for (std::vector<int>::iterator it = popTriangle.neighbours.begin() ; it != popTriangle.neighbours.end(); ++it)
	{
	  if(T[*it].lable == -1){
	    T[*it].tag = popTriangle.tag;
	    T[*it].err = distortionError(p[popTriangle.tag], T[*it]);
	    errQue.push_back(T[*it]);
	  }
	}
    } // popTriangle.lable == -1
  }
  
  // for (unsigned int i = 0; i < 6; i++) 
  //   for (unsigned int j = 0; j < p[i].T.size (); j++)   
  //     cout << p[i].T[j].index << endl;
  
  
  
}
