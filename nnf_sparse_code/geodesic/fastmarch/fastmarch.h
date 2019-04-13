/***************************************************************************/
/* Triangle.c : test fast marching Eikonal on acute (non obtuse)           */ 
/* triangulated grids	   											  	   */
/* This program was written by Ronny Kimmel    						       */
/***************************************************************************/
#pragma once
#ifndef FAST_MARCH_H
#define FAST_MARCH_H

#include <fastmarchutils.h>								/* declaration file */

#include <stdlib.h>
#include <string.h> 
#include <vector>

class FMM {

public:

	FMM(const float *X, const float *Y, const float *Z, const int *TRI, int vnum, int tnum) : Vnum(vnum), Tnum(3*tnum) {
		CombineVectors(&V, &T, &NonSplitTnum, 
					X,Y,Z,TRI, Vnum, Tnum); 
        InitGraph(T,V, Vnum, NonSplitTnum, Tnum); /* init V to isolated vertices */
		heap = new Heap(Vnum);
    }
	void March(float *SourceVal, float *DistanceData) {
        heap->reset();
        MarchAlg(V, Vnum, SourceVal); //, SourceVal+Vnum);	/* Perform the march */			
		for(int i = 0; i < Vnum; i++){	/* Fill up the result matrix with data.					*/
           *(DistanceData++) = V[i].U;
        }
    }

	void MarchLimited(float *SourceVal, float *DistanceData, uint32_t* indices,uint32_t &M,float R_max) {
		heap->reset();
		MarchAlgSort(V,
			Vnum,
			SourceVal, indices, M, R_max); //, SourceVal+Vnum);	/* Perform the march */			
		for (int i = 0; i < Vnum; i++) {	/* Fill up the result matrix with data.					*/
			*(DistanceData++) = V[i].U;
		}
	}


	~FMM() {
        /* Free the resources used by the program. */
		delete heap;
        free(T);
        free(V);
    }

	int GetV() { return Vnum; }
	int GetT() { return Tnum; }

protected:

	void InitGraph(struct Triangle * T, struct Vertex * V, int Vnum, int NonSplitTnum, int Tnum);
	void CombineVectors(struct Vertex *(V[]), struct Triangle *(T[]), int *NonSplitTnum, 
						const float *X, const float *Y, const float *Z, const int *TrianglesData, 
						int Vnum, int Tnum);
	void InitMarch(float srcval[], int Vnum);
	void MarchAlg(struct Vertex  *temp_V,	
				  int temp_Vnum,
				  float *srcval);
	void MarchAlgSort(struct Vertex  *temp_V,
		int temp_Vnum,
		float *srcval, uint32_t* indices, uint32_t &M, float R_max);

	void __forceinline __fastcall FMM::CallTUpdateNeighbors(int             i,	/* vertex number */
															int start_from		/* The source from which the current march began.	*/
															);
	__forceinline void TUpdateNeighbors(
		int    i,						/* The vertex to update				   */
		int	   becomes_alive			/* The vertex that now becomes alive.  */
	);
	__forceinline float __fastcall update(struct Stencil *current_stencil, int k, int l);

private:

	struct Triangle *T; 	/* triangles, numbered 1..Tnum */
	struct Vertex   *V; 	/*  vertices coordinates 1..VNum */
	int	Tnum, Vnum;
	int NonSplitTnum;		/* The number of triangles with no splitting. */
							/* Names of files that are used for writing the results. */
	Heap *heap;

};

#endif