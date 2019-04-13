#include <pcl_fast_marching.h>

using namespace dis3;
using namespace pcl;

float* createdistancematrix(PolygonMesh::Ptr mesh) {
	XYZCloud::Ptr cloud(new XYZCloud());
	fromPCLPointCloud2(mesh->cloud, *cloud);
	int VNum = cloud->size();
	int TNum = mesh->polygons.size();
	Eigen::MatrixXf D(VNum,VNum);
	vector<float> X(VNum), Y(VNum), Z(VNum);
	vector<int> T(TNum*3);
	vector<float> SourceVal(VNum, INFINITY);
	float *DistanceData = (float *)calloc(VNum*VNum, sizeof(float));
	for (int i = 0; i < VNum; i++) {
		X.at(i) = cloud->at(i).x;
		Y.at(i) = cloud->at(i).y;
		Z.at(i) = cloud->at(i).z;
	}

	for (int i = 0; i < TNum; i++) {
		T.at(i) = mesh->polygons.at(i).vertices.at(0);
		T.at(i+TNum) = mesh->polygons.at(i).vertices.at(1);
		T.at(i+2*TNum) = mesh->polygons.at(i).vertices.at(2);
	}
	FMM *fmm = new FMM(X.data(), Y.data(), Z.data(), T.data(), VNum, TNum);
	for (int i = 0; i < VNum; i++) {
		SourceVal.at(i) = 0;
		fmm->March(SourceVal.data(), DistanceData + i * VNum);
		SourceVal.at(i) = INFINITY;
	}
	return DistanceData;
}


vector<vector<float>> createdistancematrixOMP(PolygonMesh::Ptr mesh,int threads) {
	XYZCloud::Ptr cloud(new XYZCloud());
	fromPCLPointCloud2(mesh->cloud, *cloud);
	int VNum = cloud->size();
	int TNum = mesh->polygons.size();
	Eigen::MatrixXf D(VNum, VNum);
	vector<float> X(VNum), Y(VNum), Z(VNum);
	vector<int> T(TNum * 3);
	vector<vector<float>> DistanceData(VNum, vector<float>(VNum));
	//double *DistanceData = (double *)calloc(VNum*VNum, sizeof(double));
	for (int i = 0; i < VNum; i++) {
		X.at(i) = cloud->at(i).x;
		Y.at(i) = cloud->at(i).y;
		Z.at(i) = cloud->at(i).z;
	}
	int ind_per_thread = floor(VNum / threads);
	vector<int> end_ind;
	for (int t = 0; t < threads; t++) {
		end_ind.push_back((t + 1)*ind_per_thread);
	}
	end_ind.at(threads-1) = VNum;
	for (int i = 0; i < TNum; i++) {
		T.at(i) = mesh->polygons.at(i).vertices.at(0);
		T.at(i + TNum) = mesh->polygons.at(i).vertices.at(1);
		T.at(i + 2 * TNum) = mesh->polygons.at(i).vertices.at(2);
	}
	omp_set_num_threads(threads);

#pragma omp parallel
	{

		cout << omp_get_num_threads();
#pragma omp for
		for (int t = 0; t < threads; t++) {
			cout << "thread number" << omp_get_thread_num() << endl;
			vector<float> SourceVal(VNum, INFINITY);
			FMM *fmm = new FMM(X.data(), Y.data(), Z.data(), T.data(), VNum, TNum);
			int start = t * ind_per_thread, end = end_ind.at(t);
			for (int i = start; i < end; i++) {
				SourceVal.at(i) = 0;
				//fmm->March(SourceVal.data(), DistanceData + i * VNum);
				fmm->March(SourceVal.data(), DistanceData.at(i).data());

				SourceVal.at(i) = INFINITY;
			}
			delete fmm;
		}
	}
	for (int i = 0; i < VNum - 1; i++) {
		for (int j = i + 1; j < VNum; j++) {
			float val = min(DistanceData.at(i).at(j), DistanceData.at(j).at(i));
			DistanceData.at(i).at(j) = val; DistanceData.at(j).at(i) = val;
		}
	}

	return DistanceData;
}

vector<vector<float>> FastMeshOMP::createdistancematrixOMP(int threads) {
	vector<vector<float>> DistanceData(VNum, vector<float>(VNum));
	int ind_per_thread = floor(VNum / threads);
	vector<int> end_ind;
	for (int t = 0; t < threads; t++) {
		end_ind.push_back((t + 1)*ind_per_thread);
	}
	end_ind.at(threads - 1) = VNum;
#pragma omp parallel for
	for (int t = 0; t < threads; t++) {
		vector<float>& SourceVal = SourceVal_t.at(t);
		FMM* fmm = fmm_t.at(t);
		cout << "thread number" << omp_get_thread_num() << endl;
		int start = t * ind_per_thread, end = end_ind.at(t);
		for (int i = start; i < end; i++) {
			SourceVal.at(i) = 0;
			//fmm->March(SourceVal.data(), DistanceData + i * VNum);
			fmm->March(SourceVal.data(), DistanceData.at(i).data());
			SourceVal.at(i) = INFINITY;
		}
	}

	for (int i = 0; i < VNum - 1; i++) {
		for (int j = i + 1; j < VNum; j++) {
			float val = min(DistanceData.at(i).at(j), DistanceData.at(j).at(i));
			DistanceData.at(i).at(j) = val; DistanceData.at(j).at(i) = val;
		}
	}
	return DistanceData;
}

void FastMeshOMP::GeodesicDisc(uint32_t src, vector<ind_r_s>& disc, uint32_t& M, float R_Disc, uint8_t thread) {
	vector<float>& SourceVal = SourceVal_t.at(thread);
	vector<float>& DistanceData = Distances_t.at(thread);
	vector<uint32_t>& indices = indices_t.at(thread);
	FMM* fmm = fmm_t.at(thread);
	SourceVal.at(src) = 0;
	fmm->MarchLimited(SourceVal.data(), DistanceData.data(), indices.data(),M, R_Disc);
	SourceVal.at(src) = INFINITY;
	for (int i = 0; i < M; i++) {
		uint32_t ind = indices.at(i);
		disc.at(i) = ind_r_s(ind, DistanceData.at(ind));
	}

}
/*vector<ind_r_s> getGeoDisc(PolygonMesh::Ptr mesh, uint32_t ind, float R_thresh) {
SourceVal.at(i) = 0;


}*/