#include <fastmarchutils.h>								/* declaration file */

float L2(Vertex *V, int Vnum)
{
	int             i;
	float          d, x, y, z;
	float          sum = 0, dxdy = 1.0/((float)Vnum);
	/* find the zero point */
	for (i = 0; i < Vnum; i++)
		if (V[i].U < 0.00000001) {
			x = V[i].x;
			y = V[i].y;
			z = V[i].z;
		}
	for (i = 0; i < Vnum; i++) {
		d = sqrt(Sqr(x - V[i].x) + Sqr(y - V[i].y) + Sqr(z - V[i].z));
		sum += Sqr(V[i].U - d)*dxdy; 
	}
	return (sqrt(sum));
}
/***************************************************************************/
/* L1  error norm the diff between the distance and U*/
/***************************************************************************/
float L1(Vertex *V, int Vnum)
{
	int             i;
	float          d, x, y, z;
	float          sum = 0.0, dxdy = 1.0/((float)Vnum);
	/* find the zero point */
	for (i = 0; i < Vnum; i++)
		if (V[i].U < 0.00000001) {
			x = V[i].x;
			y = V[i].y;
			z = V[i].z;
		}
	for (i = 0; i < Vnum; i++) {
		d = sqrt(Sqr(x - V[i].x) + Sqr(y - V[i].y) + Sqr(z - V[i].z));
		sum += ABS(V[i].U - d)*dxdy;
	}
	return (sum);
}

/***************************************************************************/
/* CosAngle the cos of the angle at the vertex v0, between v1 and v2	   */
/***************************************************************************/
float CosAngle(int v0,int v1,int v2,struct Vertex *V)
{
	float x1,x2,y1,y2,z1,z2,res;
	if(v0 != -1 && v1 != -1 && v2 != -1){
		x1 = V[v1].x - V[v0].x;
		x2 = V[v2].x - V[v0].x;
		y1 = V[v1].y - V[v0].y;
		y2 = V[v2].y - V[v0].y;
		z1 = V[v1].z - V[v0].z;
		z2 = V[v2].z - V[v0].z;
		res = x1*x2+y1*y2+z1*z2;		/* dot product */
		res /= sqrt(x1*x1+y1*y1+z1*z1); /* normalize */
		res /= sqrt(x2*x2+y2*y2+z2*z2);
		return(res);
	}
	else
		return 0;
}
/***************************************************************************/
/* Length between the vertex v0 and v1									   */
/***************************************************************************/
float
Length(int v0,int v1,struct Vertex *V)
{
	float x1,y1,z1,res;
	if(v0 != -1 && v1 != -1){
		x1 = V[v1].x - V[v0].x;
		y1 = V[v1].y - V[v0].y;
		z1 = V[v1].z - V[v0].z;
		res = sqrt(x1*x1+y1*y1+z1*z1); /* distance */
		return(res);
	}
	else
		return MAX_DISTANCE;
}
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
      struct Triangle * T, struct Vertex * V, int Tnum)
{
	bool				found = FALSE;	/* Indicates whether we found the next triangle. */
	int					i,				/* Index for the loop.							 */
						tj,				/* A candidate tp be the next triangle.			 */
						k;				/* Index for the inner loop.					 */
	/* scan every triangle of vi */
	for (i = 0; i < V[v1].ti && ! found; i++) {
		tj = V[v1].Tind[i];
		if (tj < Tnum && tj != ti)
			/* search for tj in the list of v2 */
			for (k = 0; k < V[v2].ti && ! found; k++) 
				if (V[v2].Tind[k] == tj && !T[tj].Split) {
					found = TRUE;
					*tn = tj;
				}
	}
	if (found){ /* find v3, the other vertex in the triangle to be unfolded.			 */
		if(T[*tn].Vind[0] == v1){
			if(T[*tn].Vind[1] == v2)
				*v3 = T[*tn].Vind[2];
			else
				*v3 = T[*tn].Vind[1];
		}
		else if(T[*tn].Vind[1] == v1){
			if(T[*tn].Vind[0] == v2)
				*v3 = T[*tn].Vind[2];
			else
				*v3 = T[*tn].Vind[0];
		}
		else{
			if(T[*tn].Vind[0] == v2)
				*v3 = T[*tn].Vind[1];
			else
				*v3 = T[*tn].Vind[0];
		}
	}
	return (found);
}

/***************************************************************************/
/* Split obtuse angles by unfolding splitting and connecting, return the   */
/* number of unfoldings that were nessesery.							   */
/* ti is the tirnalge to be splittined, V0 is the vertex with the obtuse   */
/* angle while V1 and V2 are the other vertexes of ti.					   */
/***************************************************************************/
int
Split(int ti, int V0, int V1, int V2, struct Triangle * T, struct Vertex * V,
      int NonSplitTnum, int Vnum)
{
	float          xv1,x1, y1, yv1,
					x2,				/* The distance between V0 and V2 */
					y2,
					x3,y3, xt2, xt3, yt3,
					e0,				/* The distance between v1 and v2 */
					e1,				/* The distance between v2 and v3.*/
					e2,				/* The distance between v0 and v1 */
					ta,				/* Tan of alpha.				  */
					cb, sb;			/* Cos and Sin of beta.			  */
	int             v1 = V1, v2 = V2, v3,
					tm=ti,			/* The current triangle we are
									   working on.					  */
					tn,				/* The triangle returned by NextT */
					count = 0;		/* The number of triangles unfolded
									   so far.						  */
									/* Becomes true when the split was done */
	bool         splitter = FALSE;
	x2 = Length(V0, V2, V);
	y2 = 0;
	e0 = Length(V1, V2, V);
	e2 =  Length(V0, V1, V);
	xv1 = x1 = (x2 * x2 + e2 * e2 - e0 * e0) / (2.0 * x2);/* translation */
	yv1 = y1 = sqrt(e2 * e2 - x1 * x1);
	ta = -x1 / y1;		/* tan (alpha) in Fig. 1 */
	/* if there is a next triangle and not splited */
	while (nextT(tm, v1, v2, &v3, &tn, T, V, NonSplitTnum) && (! splitter) ) {
		count++;
		tm = tn;		/* Update the wording triangle. */
		cb = (x2 - x1) / sqrt(Sqr(x2 - x1) + Sqr(y2 - y1));	/* cos beta */
		sb = sqrt(1 - cb * cb);								/* sin beta */
		if (y2 < y1)	/* Adjast the sign of SIN(beta).				*/
			sb *= -1.0;
		xt2 = Length(v1, v2, V);
		e1 = Length(v2, v3, V);
		e2 = Length(v1, v3, V);
		xt3 = (xt2 * xt2 + e2 * e2 - e1 * e1) / (2.0 * xt2);
		yt3 = sqrt(e2 * e2 - xt3 * xt3);
		x3 = cb * xt3 - sb * yt3 + x1;
		y3 = sb * xt3 + cb * yt3 + y1;

		if (x3 > 0 && y3/x3 > ta) {		/* if we found a splitter */
			splitter = TRUE;
											/* Add the stencils involving the
											   splitting edge.			*/
			V[V0].ST[V[V0].si].Ctheta = (x3*xv1+y3*yv1)/
				sqrt((xv1*xv1+yv1*yv1)*(x3*x3+y3*y3));
			V[V0].ST[V[V0].si].v1 = V1;
			V[V0].ST[V[V0].si].v2 = v3;
			V[V0].ST[V[V0].si].l1 = Length(V0, V1, V);
			if(V[V0].si == MaxTri - 1) {
			//	mexPrintf("Warning, too many triangles to one vertex, result quality will suffer\n");
			}
			else
				V[V0].ST[V[V0].si++].l2 = sqrt(x3*x3+y3*y3);


			V[V0].ST[V[V0].si].Ctheta = x3/sqrt(x3*x3+y3*y3);
			V[V0].ST[V[V0].si].v1 = v3;
			V[V0].ST[V[V0].si].v2 = V2;
			V[V0].ST[V[V0].si].l1 =sqrt(x3*x3+y3*y3);
			if(V[V0].si == MaxTri - 1) {
			//	mexPrintf("Warning, too many triangles to one vertex, result quality will suffer\n");
			}
			else
				V[V0].ST[V[V0].si++].l2 =Length(V0, V2, V);

			if(V[v3].vn == 3 * MaxTri - 1) {
			//	mexPrintf("Warning, too many triangles to one vertex, result quality will suffer\n");
			}
			else
				V[v3].VN[V[v3].vn++] = V0; /* add dirrectional edge
											  to v3					*/

			T[NonSplitTnum + (ti * 2)].Vind[0] = V1;	/* Add the triangles of the splitting. */
			T[NonSplitTnum + (ti * 2)].Vind[1] = V0;
			T[NonSplitTnum + (ti * 2)].Vind[2] = v3;
			T[NonSplitTnum + (ti * 2)].Split = TRUE;
			if(V[V1].ti == MaxTri - 1) {
			//	mexPrintf("Warning, too many triangles to one vertex, result quality will suffer\n");
			}
			else
				V[V1].Tind[V[V1].ti++] = NonSplitTnum + (ti * 2);
			if(V[V0].ti == MaxTri - 1) {
			//	mexPrintf("Warning, too many triangles to one vertex, result quality will suffer\n");
			}
			else
				V[V0].Tind[V[V0].ti++] = NonSplitTnum + (ti * 2);
			if(V[v3].ti == MaxTri - 1){
			//	mexPrintf("Warning, too many triangles to one vertex, result quality will suffer\n");
			}
			else
				V[v3].Tind[V[v3].ti++] = NonSplitTnum + (ti * 2);
			T[NonSplitTnum + (ti * 2) + 1].Vind[0] = v3;
			T[NonSplitTnum + (ti * 2) + 1].Vind[1] = V0;
			T[NonSplitTnum + (ti * 2) + 1].Vind[2] = V2;
			T[NonSplitTnum + (ti * 2) + 1].Split = TRUE;
			if(V[v3].ti == MaxTri - 1) {
			//	mexPrintf("Warning, too many triangles to one vertex, result quality will suffer\n");
			}
			else
				V[v3].Tind[V[v3].ti++] = NonSplitTnum + (ti * 2) + 1;
			if(V[V0].ti == MaxTri - 1) {
			//	mexPrintf("Warning, too many triangles to one vertex, result quality will suffer\n");
			}
			else
				V[V0].Tind[V[V0].ti++] = NonSplitTnum + (ti * 2) + 1;
			if(V[V2].ti == MaxTri - 1) {
			//	mexPrintf("Warning, too many triangles to one vertex, result quality will suffer\n");
			}
			else
				V[V2].Tind[V[V2].ti++] = NonSplitTnum + (ti * 2) + 1;


		}
		else {						   /* we have not found a splitter,
										  continue unfolding			*/
			if (x3 < 0){
				v1 = v3; x1 = x3; y1 = y3;
			} else {
				v2 = v3; x2 = x3; y2 = y3;
			}
		}	
	}
	return(count);				/* Return the number of triangles that were
								   unfolded.							   */
}
