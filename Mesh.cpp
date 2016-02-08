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
  partitioning();

  for (unsigned int i = 0; i < iteNum; i++)
    {
      proxyFitting();
      partitioning();
    }

  vertexClass();
  edgeExtraction();
  triangulation();
  insertTriangle();
  //  remesh();
  // vec3f a(0.0,3.0,0.0);
  // Vec3f b(4.0,0.0,0.0);
  // Vec3f c(0.0,0.0,0.0);
  // Proxy p(a,b);
  // distortionError(p,a,b,c);
}

void Mesh::recomputeNormals () {
  for (unsigned int i = 0; i < V.size (); i++){
    V[i].n = Vec3f (0.0, 0.0, 0.0);
    V[i].index = i;
  }
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


//Triangle Mesh::popLeastErrTriangle(std::vector<Triangle > &errQue){
Triangle Mesh::popLeastErrTriangle(){
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


void Mesh::proxyFitting() {

  int sum = 0;

  for (unsigned int i = 0; i < num; i++)
    {
      Vec3f X = Vec3f(0.0, 0.0, 0.0);
      Vec3f N = Vec3f(0.0, 0.0, 0.0);

      for (std::vector<Triangle >::iterator it = p[i].T.begin() ; it != p[i].T.end(); ++it)
	{
	  float area = 0.0;
	  area = 0.5 * (*it).n.length();
	  N += area * (*it).n;
	  X += (V[(*it).v[0]].p + V[(*it).v[1]].p + V[(*it).v[2]].p);
	}
      
      N.normalize();
      sum += p[i].T.size();
      //cout << p[i].T.size() << endl;
      // cout << N << endl;
      // change the proxy normal and center
      p[i].n = N;
      if(!p[i].T.empty())
	p[i].x = X/(3*p[i].T.size());
    }

  cout << sum << endl;
  
  // clear all flag
  for (unsigned int i = 0; i < T.size (); i++)
    {
      T[i].tag = -1.0;
      T[i].lable = -1.0;
      T[i].err = -1.0;
    }
  
  
  /// find seed triangle  
  for (unsigned int i = 0; i < num; i++)
    {
      float minerr = 1000.0;
      float Terr= 0.0;
      int  minIndex = 0;
      
      for (std::vector<Triangle >::iterator it = p[i].T.begin() ; it != p[i].T.end(); ++it)
	{
	  Terr = distortionError(p[i], *it);
	  if( Terr < minerr) {
	    minerr = Terr; 
	    minIndex = it->index; 
	  }
	}
  
      T[minIndex].lable = i;
      // cout << minIndex << endl;
      //clear all triangles
      p[i].T.clear();
      p[i].T.push_back(T[minIndex]);
    }
  
  
}


bool Mesh::repeatedNum(int testNum){
  for (unsigned int i = 0; i < num; i++){
    if( seed[i] == testNum ) 
      return true;
  }
  
  return false;
}

void Mesh::partitioning() {

  int initSeed;
  Triangle popTriangle;

  /* initialize random seed: */
  srand (time(NULL));
  
  /* generate secret number between 0 and T.size() - 1: */
  if(flagFirst == true){
    for (unsigned int i = 0; i < num; i++){
      initSeed = rand() % T.size();
      if(repeatedNum(initSeed))	i--;
      else seed[i]= initSeed;
    }

#if debug == 1
    for (unsigned int i = 0; i < num; i++)
      cout << seed[i] <<' ';
    cout << endl;
#endif

    for (unsigned int i = 0; i < num; i++){
      p[i].T.push_back(T[seed[i]]);
      T[seed[i]].lable = i; 
      p[i].x = (V[p[i].T[0].v[0]].p + V[p[i].T[0].v[1]].p + V[p[i].T[0].v[2]].p)/3;
      p[i].n = p[i].T[0].n;
    }
    
    flagFirst = false;
  }
  
#if debug == 1
  for (unsigned int i = 0; i < num; i++){
    cout <<"Init Proxy Barycenter: " << p[i].x << endl;
    cout << "Init Proxy Normal: "<< p[i].n << endl;
    cout << seed[i] << endl;
    //see wheather initialization is sucessful
    cout << p[i].T.size() << endl;
    cout << p[i].T[0].lable<< endl;
    cout << p[i].T[0].tag<< endl;
    cout << p[i].T[0].err<< endl;
    cout << p[i].T[0].neighbours.size() << endl;    
  }
#endif
  
  // initialization for the error queque
  for (unsigned int i = 0; i < num; i++)
    {
      // initalize tag and lable
      for (std::vector<int>::iterator it = p[i].T[0].neighbours.begin() ; it != p[i].T[0].neighbours.end(); ++it)
	{
	  T[*it].tag = i;
	  T[*it].err = distortionError(p[i], T[*it]);
	  // std::cout << ' ' << T[*it].err;
	  errQue.push_back(T[*it]);
	}
      //      std::cout << '\n';
    }



  //  std::cout <<"Initial Queque Size :" << errQue.size() << endl;
  while(!errQue.empty()){
    //std::cout <<"Initial Queque Size :" << errQue.size() << endl;
    /// pop the triangle with the smallest error
    popTriangle = popLeastErrTriangle();


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
  
  
  
}


float Mesh::distanceToLine(Vertex & V, anchorPair & P){
  Vec3f p0 = V.p;
  Vec3f p1 = P.anchorVertex[0].p;
  Vec3f p2 = P.anchorVertex[1].p;
  Vec3f ni = p[P.proxy[0]].n;
  Vec3f nj = p[P.proxy[1]].n;

  
  float distance = cross((p0 - p1), (p0 - p2)).length() / (p1 - p2).length();
  float judge = distance * (cross(ni, nj).length()/(ni.length() * nj.length()))/dist(p1, p2);

  //cout << judge << endl;

  // only these vertex which greater than treshold can pass
  if(judge > threshold)
    return distance;
  else 
    return 0.0;
}

void Mesh::findMoreAnchor(){
  anchorPair temp, temp1;
  
  for (unsigned int i = 0; i < anchorP.size() ; i++){
    float maxD = -1.0;
    float distance = 0.0;
    int maxIndex = 0;
    
    for(unsigned int j = 0; j < anchorP[i].edges.size() ; j++){
      distance = distanceToLine(anchorP[i].edges[j], anchorP[i]);
      if(distance > maxD){
	maxD = distance;
	maxIndex = j;
      }
    }
 
    if(maxD > 0.0){
      //cout << "!" << endl;
      anchorV.push_back(anchorP[i].edges[maxIndex]);
      
      
      temp.anchorVertex[0] = anchorP[i].edges[maxIndex];
      temp.anchorVertex[1] = anchorP[i].anchorVertex[0];
      temp.proxy[0] = anchorP[i].proxy[0];
      temp.proxy[1] = anchorP[i].proxy[1];
      anchorPCopy.push_back(temp);

      temp1.anchorVertex[0] = anchorP[i].edges[maxIndex];
      temp1.anchorVertex[1] = anchorP[i].anchorVertex[1];
      temp1.proxy[0] = anchorP[i].proxy[0];
      temp1.proxy[1] = anchorP[i].proxy[1];
      anchorPCopy.push_back(temp1);
            
      anchorPCopy[i].proxy[0] = -1 ;
      anchorPCopy[i].proxy[1] = -1 ;
    }
    
  }
  
}


bool Mesh::isLine(Vertex & V0, Vertex & V1, int * p){
  int cnt = 0;

  for (unsigned int i = 0; i < V0.proxies.size() ; i++){
    for (unsigned int j = 0; j < V1.proxies.size() ; j++){
      if(V0.proxies[i] ==  V1.proxies[j]){
	p[cnt] = V0.proxies[i];
	cnt++;
      }
    }
  }
  
  if(cnt == 2) return true;
  else return false;
}

bool Mesh::isinLine(Vertex & V, anchorPair & P){
  
  if((V.proxies[0] == P.proxy[0] && V.proxies[1] == P.proxy[1]) 
     || (V.proxies[1] == P.proxy[0] && V.proxies[0] == P.proxy[1]))
      return true;
    else return false;
}

void Mesh::edgeExtraction(){

  //  std::vector<Vertex> anchorV;
  //  std::vector<Vertex> edgeV;
  //  std::vector<anchorPair> anchorP;
  anchorPair temp;
  
  for (unsigned int i = 0; i < V.size() ; i++){
    if(V[i].proxies.size() >= 3)   anchorV.push_back(V[i]);
    else if(V[i].proxies.size() == 2)   edgeV.push_back(V[i]);
  }
  
  for (unsigned int i = 0; i < anchorV.size() ; i++){
    for(unsigned int j = i + 1; j < anchorV.size() ; j++){
      if(isLine(anchorV[i], anchorV[j], temp.proxy)){
	temp.anchorVertex[0] = anchorV[i];
	temp.anchorVertex[1] = anchorV[j];
	anchorP.push_back(temp);
      }
    }
  }

  
#if debug == 1
  for (unsigned int i = 0; i < anchorV.size() ; i++){
    for(unsigned int j = 0; j < anchorV[i].proxies.size() ; j++){
      cout << anchorV[i].proxies[j] << ' ';
    }
    cout << endl;
  }
  
  for (unsigned int i = 0; i < edgeV.size() ; i++){
    for(unsigned int j = 0; j < edgeV[i].proxies.size() ; j++){
      cout << edgeV[i].proxies[j] << ' ';
    }
    cout << endl;
  }

  for (unsigned int i = 0; i < anchorP.size() ; i++){
    
    for(unsigned int j = 0; j < anchorP[i].anchorVertex[0].proxies.size() ; j++){
      cout << anchorP[i].anchorVertex[0].proxies[j] << ' ';
    }
    cout << endl;

    for(unsigned int j = 0; j < anchorP[i].anchorVertex[0].proxies.size() ; j++){
      cout << anchorP[i].anchorVertex[1].proxies[j] << ' ';
    }
    
    cout << anchorP[i].proxy[0] << " + " << anchorP[i].proxy[1] << endl;
  }
  
  cout << endl;
#endif
  
  // find the edge vertex to pair line 
  for (unsigned int i = 0; i < anchorP.size() ; i++){
    for (unsigned int j = 0; j < edgeV.size() ; j++){
      if(isinLine(edgeV[j], anchorP[i])){
	anchorP[i].edges.push_back(edgeV[j]);
      }
    }
  }

  
#if debug == 1
  for (unsigned int i = 0; i < anchorP.size() ; i++){
    cout << anchorP[i].edges.size() <<endl;  
    for(unsigned int j = 0; j < anchorP[i].edges.size() ; j++){

      for (std::vector<int>::iterator it = anchorP[i].edges[j].proxies.begin() ; it != anchorP[i].edges[j].proxies.end(); ++it)
	cout << *it << ' ';
    }
      cout << endl;  
  }
#endif

  //save edges
  anchorPCopy = anchorP;

  if(edgeExtra)
    findMoreAnchor();  
  

  //  find the anchor vertex for each proxy
  for (unsigned int i = 0; i < anchorV.size() ; i++){
    //    anchorV[i].lable = i;//!!!
    for(std::vector<int>::iterator it = anchorV[i].proxies.begin(); it != anchorV[i].proxies.end(); ++it){
      p[*it].anchorV.push_back(anchorV[i]);
    }
  }
  

  //  find the inner vertex for each proxy
  for (unsigned int i = 0; i < V.size() ; i++){
    for(std::vector<int>::iterator it = V[i].proxies.begin(); it != V[i].proxies.end(); ++it){
      p[*it].V.push_back(V[i]);
    }
  }

  //  find the  vertex pair for each proxy
  if(edgeExtra){
    for (unsigned int i = 0; i < anchorPCopy.size() ; i++){
      if(anchorPCopy[i].proxy[0] > 0 ){
	p[anchorPCopy[i].proxy[0]].anchorP.push_back(anchorPCopy[i]);
	p[anchorPCopy[i].proxy[1]].anchorP.push_back(anchorPCopy[i]);
      }
    }
  }
  else{
    for (unsigned int i = 0; i < anchorP.size() ; i++){
      p[anchorP[i].proxy[0]].anchorP.push_back(anchorP[i]);
      p[anchorP[i].proxy[1]].anchorP.push_back(anchorP[i]);
    }
  }

#if debug == 1
  for (unsigned int i = 0; i < num ; i++){
    cout << p[i].anchorV.size() << " : ";
    cout << p[i].V.size() << " : ";
    cout << p[i].anchorP.size() << endl;
  }
#endif  
  
}

/// find triangle whose vertex have three different colors
void Mesh::insertTriangle(){
  
  int color[3] = {-1, -1, -1};
  Triangle temp;
    

  for (unsigned int i = 0; i < num ; i++){
    for(std::vector<Vertex>::iterator it = p[i].V.begin(); it != p[i].V.end(); ++it){
      V[(*it).index].lable = (*it).lable;
      //      cout << (*it).lable << ' ';
    }
    
  cout << endl;
	
    for(std::vector<Triangle>::iterator it = p[i].T.begin(); it != p[i].T.end(); ++it){
      for (unsigned int k = 0; k < 3; k++) 
	color[k] = V[(*it).v[k]].lable;
      
      //cout << color[0] <<" *" << color[1] <<"*" << color[2] <<endl;
      
      if(color[0] != color[1] && color[0] != color[2] && color[1] != color[2]){
	cout << "Proxy " << i << " add a triangle! : ";
	cout << color[0] <<' ';
	cout << color[1] <<' ';
	cout << color[2] <<' '<<endl;
	
	temp = *(it);
	for (unsigned int j = 0; j < 3; j++) 
	  temp.v[j] = p[i].anchorV[color[j]].index;

	for (unsigned int j = 0; j < p[i].anchorP.size(); j++) {
	  if((temp.v[0] == p[i].anchorP[j].anchorVertex[0].index || temp.v[1] == p[i].anchorP[j].anchorVertex[0].index || temp.v[2] == p[i].anchorP[j].anchorVertex[0].index) && (temp.v[0] == p[i].anchorP[j].anchorVertex[1].index || temp.v[1] == p[i].anchorP[j].anchorVertex[1].index || temp.v[2] == p[i].anchorP[j].anchorVertex[1].index))
	    p[i].anchorP[j].used = 1;
	}
	
	TR.push_back(temp);  
      }
    }

    // test
    cout << "Proxy " << i << " has anchor vertex: ";
    for (unsigned int j = 0; j < p[i].anchorV.size(); j++) {
      cout << p[i].anchorV[j].lable << ' ';
    }
    cout << endl;
    
    for (unsigned int j = 0; j < p[i].anchorP.size(); j++) {
      if(p[i].anchorP[j].used < 0){
	cout <<"Proxy " << i <<" miss edge :";
	
	cout << V[p[i].anchorP[j].anchorVertex[0].index].lable << ' ';
	cout << V[p[i].anchorP[j].anchorVertex[1].index].lable << endl;
      }
    }
    

  }  


}



void Mesh::triangulation(){
  //initialization for anchor vertex each proxy
  for (unsigned int i = 0; i < num ; i++){
    for (unsigned int j = 0; j < p[i].anchorV.size() ; j++)
      p[i].anchorV[j].lable = j;
  }
  
  for (unsigned int i = 0; i < num ; i++){
    for(std::vector<Vertex>::iterator it = p[i].V.begin(); it != p[i].V.end(); ++it){
      Vec3f p0 = (*it).p;
      float minDis = 1000.0;
      int lable = -1;

      for (unsigned int j = 0; j < p[i].anchorV.size() ; j++){
      	Vec3f p1 = p[i].anchorV[j].p;

      	float distance = dist(p0, p1);
	// same num with anchor vertex
	// if(distance == 0)
	//   cout << "O" << ' ';
      	if( distance < minDis ){
      	  minDis = distance;
      	  lable = p[i].anchorV[j].lable;
      	}
	
      }
      
      (*it).lable = lable;
      
    }
    //    cout << endl;
  }

#if debug == 1
    for (unsigned int i = 0; i < num ; i++){
      for(std::vector<Vertex>::iterator it = p[i].V.begin(); it != p[i].V.end(); ++it){
	cout << (*it).lable << ' ';
      }
      cout << endl;
    }
#endif


}





void Mesh::vertexClass(){
  
  std::vector<vector<int> > classVectex(V.size());
  int flag = 1;
  
  for (unsigned int i = 0; i < T.size (); i++)
    {
      for(unsigned int j = 0; j < 3; j++){

	int index = T[i].v[j];
	// cout << i << " "<< j << "*";
	// cout << index << ": "<< classVectex[index].size() << ' ';
	
	if(classVectex[index].size()){
	  
	  for(unsigned int k = 0; k < classVectex[index].size(); k++){
	    if(classVectex[index][k] == T[i].lable)
	      flag = 0;
	  }
	  
	  if(flag == 1)
	    classVectex[index].push_back(T[i].lable) ;
	  
	  flag = 1;
	  
	}
	else
	  classVectex[index].push_back(T[i].lable) ;
      }
    }
  
  
  for (unsigned int i = 0; i < V.size() ; i++){
    V[i].proxies = classVectex[i] ;
  }
  
  
}

