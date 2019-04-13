/***************************************************************************/
/* Triangle.c : test fast marching Eikonal on acute (non obtuse)           */ 
/* triangulated grids	   											  	   */
/* This program was written by Ronny Kimmel    						       */
/***************************************************************************/


#include <fastmarch.h>									/* declaration file */

/***************************************************************************/
/* InitGraph init the numerical graph V, unfold in order to split obtuse   */
/* triangles.															   */
/***************************************************************************/
void
FMM::InitGraph(struct Triangle * T, struct Vertex * V, int Vnum, int NonSplitTnum, int Tnum)
{
	int             i, k,			/* Indexes for the loops.				*/
					ti,				/* Index for the current triangle.		*/
					v1,v2,			/* Indexes for the neighbours in the
									   triangle of the the current vertex.	*/
					count= 0,		/* The number of unfolding in one split.*/
					mcount=0;		/* The maximum value count got.			*/
	bool         found;			/* Used for adding the neighbours to
									   every triangle.						*/
	float          ca;				/* The cosin value that determine
									   whether this triangle is obtuse.		*/
	struct Stencil *p_st;			/* Pointer to a stencil.				*/
	int ind, si_count;				/* Used for precalculting values for the
									   stencil.								*/


	/* Initialize the vertixes. */
	for (i = 0; i < Vnum; i++) {	/* zero counters of all vertices */
		V[i].si = 0;				/* # of connections to other triangles */
		V[i].vn = 0;				/* # of connections to other vertices */
	}

	/* Set the split field of the triangles that exist now, before the splitting to false.	*/
	for(i = 0; i < NonSplitTnum; i++)
		T[i].Split = FALSE;

	
	for (i = 0; i < Vnum; i++) {		/* scan all vertices */
		for (ti = 0; ti < V[i].ti; ti++){/* scan connected triangles */
			if (V[i].Tind[ti] < NonSplitTnum) {	/* if valid triangle */
												/* Make v1 and v2 the neighbours.			*/
				if (T[V[i].Tind[ti]].Vind[0] == i){
					v1 = T[V[i].Tind[ti]].Vind[1];
					v2 = T[V[i].Tind[ti]].Vind[2];
				}
				else if (T[V[i].Tind[ti]].Vind[1] == i){
					v1 = T[V[i].Tind[ti]].Vind[2];
					v2 = T[V[i].Tind[ti]].Vind[0];
				}
				else if (T[V[i].Tind[ti]].Vind[2] == i){
					v1 = T[V[i].Tind[ti]].Vind[0];
					v2 = T[V[i].Tind[ti]].Vind[1];
				}

				found = FALSE;					/* Add v1 as a neighbour if it is not already
												   a neighbour.								*/
				for (k = 0; k < V[i].vn; k++)
					if (v1 == V[i].VN[k])
						found = TRUE;
				if (! found)
					V[i].VN[V[i].vn++] = v1;

				found = FALSE;					/* Add v2 as a neigbour if it is not already
												   a neighbour.								*/
				for (k = 0; k < V[i].vn; k++)
					if (v2 == V[i].VN[k])
						found = TRUE;
				if (! found)
					V[i].VN[V[i].vn++] = v2;
			
				ca = CosAngle(i,v1,v2,V);
				if (ca < 0){					/* If this triangle is an obtuse angle		*/
					count = Split(V[i].Tind[ti],i,v1,v2,
						T,V,NonSplitTnum,Vnum);
					if (count > mcount)			/* Update m count.							*/
						mcount = count;
				} 
				else {							/* If no splitting was nessesery create
												   the stencil for this vertex and triangle.*/
					V[i].ST[V[i].si].Ctheta = ca;
					V[i].ST[V[i].si].v1 = v1;
					V[i].ST[V[i].si].l1 = Length(i,v1,V);
					V[i].ST[V[i].si].v2 = v2;
					V[i].ST[V[i].si].l2 = Length(i,v2,V);
					V[i].si++;
				}
			}
		}
	}

	for(ind = 0; ind < Vnum; ind++)					/* Calculate the data for each stencil.	*/
		for(p_st = V[ind].ST, si_count = V[ind].si - 1; si_count >= 0; p_st++, si_count--){
			p_st->Stheta =				1 - Sqr(p_st->Ctheta);
			p_st->Ctheta_mul_l1_div_l2_minus_one_mul_2 =p_st->Ctheta * p_st->l1 * 2 / p_st->l2 - 2;
			p_st->Ctheta_mul_l2_div_l1_minus_one_mul_2 =p_st->Ctheta * p_st->l2 * 2 / p_st->l1 - 2;
			p_st->sqr_l1 = p_st->l1 * p_st->l1;
			p_st->sqr_l2 = p_st->l2 * p_st->l2;
			p_st->shortcut1_1 = 1 - p_st->sqr_l1 * (p_st->Ctheta_mul_l2_div_l1_minus_one_mul_2 + 1) / p_st->sqr_l2;
			p_st->shortcut1_2 = 1 - p_st->sqr_l2 * (p_st->Ctheta_mul_l1_div_l2_minus_one_mul_2 + 1) / p_st->sqr_l1;
			p_st->shortcut2_1 = - p_st->Stheta * p_st->sqr_l1;
			p_st->shortcut2_2 = - p_st->Stheta * p_st->sqr_l2;
		}

//	mexPrintf("\nNumber of unfoldings = %d\n",mcount);
}


void FMM::CombineVectors(struct Vertex *(V[]), struct Triangle *(T[]), int *NonSplitTnum, 
					const float *X, const float *Y, const float *Z, const int *TrianglesData, 
					int Vnum, int Tnum){

	int i, j;										/* Indexes for the loops.				*/
	const float *CoordinatesData[3] = {X,Y,Z};		/* Pointer to the data in the three 
													   vector arrays.						*/
	
	*NonSplitTnum = Tnum / 3;

													/* Allocate memory for both triangles and
												       vertixes.							*/
    *T = (struct Triangle *) malloc(sizeof(struct Triangle) * Tnum);
    if(*T == NULL){
		fprintf(stderr, "Out of memory for triangles - exiting.\n");
		exit(-1);
	}
	*V = (struct Vertex *)   malloc(sizeof(struct Vertex) * Vnum);
    if(*V == NULL){
		free(T);
		fprintf(stderr, "Out of memory for vertiexes - exiting.\n");
		exit(-1);
	}
	
	for(i = 0; i < Vnum; i++){						/* Move the data to V and T.			*/
		(*V)[i].x = *((float *)CoordinatesData[0]++);
		(*V)[i].y = *((float *)CoordinatesData[1]++);
		(*V)[i].z = *((float *)CoordinatesData[2]++);
	}
	for(i = 0; i < 3; i++)
		for(j = 0; j < *NonSplitTnum; j++){
			(*T)[j].Vind[i] = *((int *)TrianglesData++);
		}
		
	for(i = 0; i < Vnum; i++){						/* Add every triangle to its vertixes.	*/
		(*V)[i].ti = 0;
	}
	/* Can be greatly improved! */
	for(i = 0; i < *NonSplitTnum; i++){
		if((*T)[i].Vind[0] != *NonSplitTnum){
			(*V)[(*T)[i].Vind[0]].Tind[(*V)[(*T)[i].Vind[0]].ti++] = i;
			(*V)[(*T)[i].Vind[1]].Tind[(*V)[(*T)[i].Vind[1]].ti++] = i;
			(*V)[(*T)[i].Vind[2]].Tind[(*V)[(*T)[i].Vind[2]].ti++] = i;
		}
	}
	
	return;

}




/***************************************************************************/
/* IntiMarch procedure, Init phase of the procedure, the march is from the */
/* points specified by xplace and yplace and the index start_from that	   */
/* the algoritem gets. If the flag ALL_SOURCES_TOGETHER is defined the	   */
/* the procedure ignore the third argument and use all points in xplace	   */
/* and yplace as sources.												   */
/***************************************************************************/
void FMM::InitMarch(float srcval[], int Vnum){
	int i;
	for(i = 0; i < Vnum; i++){

		if (srcval[i] >= MAX_DISTANCE) {
			V[i].U = MAX_DISTANCE;					/* Initialize the distance				*/
			heap->BP[i] = Far;							/* Originally all are far.				*/
			V[i].source = FALSE;
			//V[i].source_num = 0;
		}
		else {
			V[i].U = srcval[i];
			heap->insert(V[i].U, i);
			V[i].source = TRUE;
			//V[i].source_num = srcnum[i];	/* Set the source num field.			*/
		}

	}


	return;

}

/***************************************************************************/
/* Quadratic.c  solving a quadratic equation qa x^2+qb x+qc=0              */
/* Only the + solution is returned.										   */
/***************************************************************************/
__forceinline bool __fastcall
Quadratic(float qa, float qb, float qc, float *x1)
{

	float d;

	d = qb*qb - (qa+qa)*(qc+qc); /* Discremenat */

    if ( d >= 0 ){  /* in case the Discremenat >= 0 */
		*x1 = (sqrt(d) - qb)/(qa + qa);
        return TRUE;
	}
    else {
        return FALSE;
    }
}

/***************************************************************************/
/* update find the solution to one triangle problem                        */
/***************************************************************************/
__forceinline float __fastcall
FMM::update(struct Stencil *current_stencil, int k, int l){

	float u1, u2;
	float u;
	float t;
	float /*ctheta_mul_mb_div_ma_minus_one_mul_2,*/ ctheta_mul_ma_div_mb_minus_one_mul_2;
																			/* Both of them are between 0 and 1 becuase
																			   the triangles are not obuse.				*/

	u1 = V[k].U;
	u2 = V[l].U;
	u = u2 - u1;
	if (u >= 0) {
		ctheta_mul_ma_div_mb_minus_one_mul_2 = current_stencil->Ctheta_mul_l2_div_l1_minus_one_mul_2;
																			/* A shortcut */
		if (Quadratic(current_stencil->shortcut1_2,							/* If the quadratic equation has solutions */
			      u * ctheta_mul_ma_div_mb_minus_one_mul_2,
			      (u * u + current_stencil->shortcut2_2), &t)
				  && (-(u + u) >= ctheta_mul_ma_div_mb_minus_one_mul_2 * t)
				  && current_stencil->Ctheta_mul_l1_div_l2_minus_one_mul_2*(t-u) <= (u + u))
				return (u1 + t);
		else									/* If the quadratic equation has no solution,
												   or the solutions are out of the trinagle */
			return MAX_DISTANCE;
	}
	else{
		ctheta_mul_ma_div_mb_minus_one_mul_2 = current_stencil->Ctheta_mul_l1_div_l2_minus_one_mul_2;
																		/* A shortcut */

		if (Quadratic(current_stencil->shortcut1_1,						/* If the quadratic equation has solutions */
			      -u * ctheta_mul_ma_div_mb_minus_one_mul_2,
			      (u * u + current_stencil->shortcut2_1), &t)
				  && (u + u >= ctheta_mul_ma_div_mb_minus_one_mul_2 * t)
				  && current_stencil->Ctheta_mul_l2_div_l1_minus_one_mul_2*(t+u) <= -(u + u))
				return (u2 + t);
		else									/* If the quadratic equation has no solution,
												   or the solutions are out of the trinagle */
			return MAX_DISTANCE;
	}


	
}

/***************************************************************************/
/* Update Neighbors procedure, update the U values of all the neighbours of*/
/* a vertex in a triangulation.											   */ 
/***************************************************************************/
__forceinline void
FMM::TUpdateNeighbors(
	int    i,						/* The vertex to update				   */
	int	   becomes_alive			/* The vertex that now becomes alive.  */
){
	float          u, t3, u0;
	int             n = heap->BP[i];
	int             k, l;
	int				st_ind;			/* Index for the stencils.				*/
	struct Stencil	*st_ptr;		/* Pointer to the current stencil.		*/


	u = V[i].U;
	u0 = u;
														/* Do for every stencil.						*/
	for (st_ptr = V[i].ST, st_ind = V[i].si; st_ind > 0; st_ind--, st_ptr++) {
														/* check all numerical connections (triangles)	*/
		k = st_ptr->v1;
		l = st_ptr->v2;
		if(k == becomes_alive || l == becomes_alive){	/* Use this stencil to update only if one k or
														   l is the new alive vertex.					*/
			if(k == becomes_alive)						/* Do the one dimensional update.				*/
				u = MIN(u, V[k].U + st_ptr->l1);
			else
				u = MIN(u, V[l].U + st_ptr->l2);
			if(heap->BP[k] == Alive || heap->BP[l] == Alive){		/* Do update only if the two other vertexes
														   of the stencil are alive, otherwise this
														   stencil will be called again anyway.		*/
				t3 = update(st_ptr, k, l);

				FAST_MIN(u, t3);						/* Minimize the distance.			*/
			}
			

//			if (u0 > u){
//				V[i].source_num = MAX(V[l].source_num, V[k].source_num);
//			}

		}
	}

//	if (V[i].source) { u = V[i].U; }

	if (n == Far){		/* not in heap                 */
		heap->insert(u, i);	/* also changes BP to Trail	   */
		V[i].U = u;
	}
	else {				/* change position in the heap */
		if (heap->a[n].u > u) {
			heap->a[n].u = V[i].U = u;
			heap->upheap(n);
		}
	}
}
/***************************************************************************/
/* Update Triangle Neighbors procedure					   */
/***************************************************************************/
void __forceinline
__fastcall FMM::CallTUpdateNeighbors(
	int             i,	/* vertex number */
	int start_from		/* The source from which the current march began.	*/
){
	int local_vn_count;			/* The number of neighbours yet to cover. */
	int *VN_ind;					/* Pointer to the array of neighbours */
										/* Call TUpdateNeighbors for every neighbour vertex of
										   the given vertex. */

//	mexPrintf("%d  (%d)\n", i,start_from);

	for (VN_ind = V[i].VN, local_vn_count = 0; local_vn_count < V[i].vn; VN_ind++, local_vn_count++)
										/* If the neighbour is not alive					*/
		if(heap->BP[*VN_ind] != Alive /*&& V[*VN_ind].source == FALSE*/){
			TUpdateNeighbors(*VN_ind, i);
		}

}

/***************************************************************************/
/* MarchAlg procedure, for fast marching method, extentions of Sethian     */
/* This procedure works with triangulations that contains only non-obtuse  */
/* triangles															   */
/***************************************************************************/
void FMM::MarchAlg(
	struct Vertex  *temp_V,	/* vertices coordinates 1..Vnum */
	int             temp_Vnum,
	float *srcval){
	V = temp_V;						/* Save the array of vertexes in the V array.		*/
	Vnum = temp_Vnum;				/* Save the number of vertexes in the array V.		*/

	InitMarch(srcval, Vnum);

									/* CAllTUpdateNeighbours for every vertex when it becomes
									   the smallest vertex in the heap. Exit the loop when
									   there are no more sources which are not alive */
	

	int j = 0;
	while (heap->N != 0){
		CallTUpdateNeighbors(heap->a[1].v, 0);
		heap->remove_top();
	}

}

void FMM::MarchAlgSort(
	struct Vertex  *temp_V,	/* vertices coordinates 1..Vnum */
	int             temp_Vnum,
	float *srcval, uint32_t* indices, uint32_t &M, float R_max) {
	V = temp_V;						/* Save the array of vertexes in the V array.		*/
	Vnum = temp_Vnum;				/* Save the number of vertexes in the array V.		*/
	M = 0;
	InitMarch(srcval, Vnum);

	/* CAllTUpdateNeighbours for every vertex when it becomes
	the smallest vertex in the heap. Exit the loop when
	there are no more sources which are not alive */


	while (heap->N != 0 && heap->a[1].u < R_max) {
		indices[M] = heap->a[1].v;
		CallTUpdateNeighbors(heap->a[1].v, 0);
		heap->remove_top();
		M++;
	}

}