#pragma once
#ifndef FAST_MARCHU_H
#define FAST_MARCHU_H

#include <y_stand.h>									/* declaration file */

float L2(Vertex *V, int Vnum);
/***************************************************************************/
/* L1  error norm the diff between the distance and U*/
/***************************************************************************/
float L1(Vertex *V, int Vnum);

/***************************************************************************/
/* CosAngle the cos of the angle at the vertex v0, between v1 and v2	   */
/***************************************************************************/
float CosAngle(int v0, int v1, int v2, struct Vertex *V);
/***************************************************************************/
/* Length between the vertex v0 and v1									   */
/***************************************************************************/
float
Length(int v0, int v1, struct Vertex *V);
/***************************************************************************/
/* nextT next triangle to be unfolded. find the triangle #, and the vertex */
/* Returns true is the next triangle was found.							   */
/* v1 and v2 indicate the edge that is common to the original triangle	   */
/* and the triangle to be unfolded.										   */
/* ti is the original triangle and v3 is the other vertex of the triangle  */
/* to be unfolded.														   */
/* vn is the index of the triangle to be unfolded.						   */
/***************************************************************************/
bool
nextT(int ti, int v1, int v2, int *v3, int *tn,
	struct Triangle * T, struct Vertex * V, int Tnum);

/***************************************************************************/
/* Split obtuse angles by unfolding splitting and connecting, return the   */
/* number of unfoldings that were nessesery.							   */
/* ti is the tirnalge to be splittined, V0 is the vertex with the obtuse   */
/* angle while V1 and V2 are the other vertexes of ti.					   */
/***************************************************************************/
int
Split(int ti, int V0, int V1, int V2, struct Triangle * T, struct Vertex * V,
	int NonSplitTnum, int Vnum);


#endif