#include <misc_utils.hpp>
using namespace std;
using namespace pcl;

float stringetof(string in) {
	return atof(in.c_str());
}

int stringetoi(string in) {
	return atoi(in.c_str());
}

int barycentric_closest(barycentric_polygon_s bar, Vertices poly) {
	int i = 0;
	float max = bar.coor[0];
	if (bar.coor[1] > max) {
		i = 1;
		max = bar.coor[1];
	}
	if (bar.coor[2] > max) {
		i = 2;
	}
	return poly.vertices[i];
};

string ftos(float f, int nd=2) {
	std::ostringstream buffer;
	
	buffer << std::fixed << std::setprecision(nd) << f;
	return buffer.str();
}

void parse_Rmode(string in, template_match_parms_s& p) {
	p.fractype = in;
	if (in.compare(0, 6, "DIRECT") == 0) {
		p.RMode = DIRECT;
	}
	else if (in.compare(0, 5, "FRACQ") == 0) {
		p.RMode = FRACQ;
	}
	else if (in.compare(0, 5, "FRACT") == 0) {
		p.RMode = FRACT;
	}
	else if (in.compare(0, 4, "FRAC") == 0) {
		p.RMode = FRAC;
	}
}

void parse_feature(string in, template_match_parms_s& p) {
	p.feature = in;
}

void parse_geo(string in, template_match_parms_s& p) {
	if (in.compare(0, 4, "FAST") == 0) {
		p.GeoMethod = FAST;
	}
	else if (in.compare(0, 4, "DIJK") == 0) {
		p.GeoMethod = DIJK;
	}
	else if (in.compare(0, 5, "DIJKP") == 0) {
		p.GeoMethod = EXTENDED;
	}
}


void parse_surface(string in, template_match_parms_s& p) {
	p.surface = in;
	string surface_dir = in;
	if (in.compare(0, 7, "POISSON") == 0) { p.T.s.Method = POISSON; p.S.s.Method = POISSON; p.surface_method = POISSON;
	}
	else if (in.compare(0, 8, "TRIANGLE") == 0) { 
		p.T.s.Method = FTRIANGLE; p.S.s.Method = FTRIANGLE; p.surface_method = FTRIANGLE;
	}
	else if (in.compare(0, 4, "LOAD") == 0) { 
		p.T.s.Method = LOAD; p.S.s.Method = LOAD; p.surface_method = LOAD;
	}
	else { p.T.s.Method = NONE; p.S.s.Method = NONE;  p.surface_method = NONE;
	}
	p.T.s.method = in; p.S.s.method = in;
}

void parse_keypoint(string in, template_match_parms_s& p) {
	p.keypoint = in;
	if (in.compare(0, 3, "ISS") == 0) {
		p.T.k.Method = ISS; p.S.k.Method = ISS;
		//p.modelResolution = 3 * T_s.MeanResolution;
	}
	else if (in.compare(0, 4, "GRID") == 0) {
		p.T.k.Method = GRID; p.S.k.Method = GRID;
	}
	else if (in.compare(0, 3, "ALL") == 0) {
		p.T.k.Method = ALL; p.S.k.Method = ALL;
	}
	else if (in.compare(0, 6, "HARRIS") == 0) {
		p.T.k.Method = HARRIS; p.S.k.Method = HARRIS;
	}
	else if (in.compare(0, 4, "SIFT") == 0) {
		p.T.k.Method = SIFT; p.S.k.Method = SIFT;
	}
	else if (in.compare(0, 5, "BOUND") == 0) {
		p.T.k.Method = BOUND; p.S.k.Method = BOUND;
		//p_t.T.k.salient_radius = feature_t_radius;
		//p_t.S.k.salient_radius = feature_o_radius;
	}
}

void parse_template(string in, template_match_parms_s& p) {
	p.patch = in;
	if (in.compare(0, 6, "SPHERE") == 0) {
		p.pMode = SPHERE;
	}
	else if (in.compare(0, 8, "GEODESIC") == 0) {
		p.pMode = GEODESIC;
	}
	else if (in.compare(0, 5, "GEONN") == 0) {
		p.pMode = GEONN;
	}
}

void parse_similarity(string in, template_match_parms_s& p) {
	p.sim = in;
	if (in.compare(0, 3, "DIS") == 0) {
		p.similarity = DIS; 
	}
	else if (in.compare(0, 4, "DDIS") == 0) {
		p.similarity = DDIS;
	}
	else if (in.compare(0, 3, "BBS") == 0) {
		p.similarity = BBS;
	}
	else if (in.compare(0, 5, "WDISP") == 0) {
		p.similarity = WDISP;
	}
	else if (in.compare(0, 4, "WDIS") == 0) {
		p.similarity = WDIS;
	}
	else if (in.compare(0, 4, "WBBS") == 0) {
		p.similarity = WBBS;
	}
	else if (in.compare(0, 5, "HDISP") == 0) {
		p.similarity = HDISP;
	}
	else if (in.compare(0, 6, "MSWDIS") == 0) {
		p.similarity = MSWDIS;
	}
	else if (in.compare(0, 7, "TRIDISP") == 0) {
		p.similarity = TRIDISP;
	}
	else if (in.compare(0, 6, "TRIDIS") == 0) {
		p.similarity = TRIDIS;
	}


	else if (in.compare(0, 4, "HDIS") == 0) {
		p.similarity = HDIS;
	}

}

void parse_nnf(string in, template_match_parms_s& p) {
	int nn = stoi(in);
	switch (nn){
	case 0: p.nnfType = NNF; break;
	case 1: p.nnfType = NNF1; break;
	case 5: p.nnfType = NNF5; p.nn = "5"; break;
	}
}

void parse_mesh(string in, template_match_parms_s& p) {
	p.mesh = in;
	if (in.compare(0, 5, "SUPER") == 0) {
		p.T.s.mt = SUPER;		p.S.s.mt = SUPER; p.meshtype = SUPER;
	}
	else if (in.compare(0, 4, "DUAL") == 0) {
		p.T.s.mt = DUAL;		p.S.s.mt = DUAL; p.meshtype = DUAL;
	}
	else if (in.compare(0, 6, "NORMAL") == 0) {
		p.T.s.mt = NORMAL;		p.S.s.mt = NORMAL; p.meshtype = NORMAL;
	}
}

void parse_r(string in, template_match_parms_s& p, nnfDest D) {
	feature_params_s& p_f = (D == Q) ? p.S.f : p.T.f;
	p_f.searchRadius = stof(in);
}

vector<string> split(const std::string& s, char delimiter)
{
	std::vector<std::string> tokens;
	std::string token;
	std::istringstream tokenStream(s);
	while (std::getline(tokenStream, token, delimiter))
	{
		tokens.push_back(token);
	}
	return tokens;
}

string get_name(string in) {
	vector<string> words = split(in,'\\');
	vector<string> words2 = split(words.at(words.size() - 1), '.');
	return words2.at(0);
}

void parse_3dis_params(char** argv, int argc, template_match_parms_s& p) {
	p.T.in = argv[1]; p.S.in = argv[2];
	p.S.path = get_name(p.S.in);// p.S.in.substr(0, p.S.in.size() - 4);
	p.T.path = get_name(p.T.in);// p.T.in.substr(0, p.T.in.size() - 4);
	p.T.s.in_path = p.T.in; p.S.s.in_path = p.S.in;
	for (int i = 3; i < argc; i++) {
		string arg = argv[i];
		if (arg.compare(0, 8, "surface=") == 0){//surface reconstruction method. usually set to load a surface
			parse_surface(arg.substr(8), p);
		} else if (arg.compare(0, 9, "keypoint=" ) == 0) {//keypoint type usually takes all points
			parse_keypoint(arg.substr(9), p);
		} else if (arg.compare(0, 8, "feature=") == 0) {//feature type
			parse_feature(arg.substr(8),p);
		} else if (arg.compare(0, 9, "template=") == 0) {//distance mode for template - geodesic or non geodesic
			parse_template(arg.substr(9),p);
		} else if (arg.compare(0, 11, "similarity=") == 0) {//similarity measure
			parse_similarity(arg.substr(11), p);
		} else if (arg.compare(0, 5, "mesh=") == 0) {
			parse_mesh(arg.substr(5), p);
		} else if (arg.compare(0, 3, "RQ=") == 0) {//radius of Template feature. not used anymore
			parse_r(arg.substr(3), p,Q);
		} else if (arg.compare(0, 3, "RT=") == 0) {//radius of Template feature. not used anymore
			parse_r(arg.substr(3), p,T);
		}
		else if (arg.compare(0, 3, "NN=") == 0) {//number of nearest neighbors calculated
			parse_nnf(arg.substr(3), p);
		}
		else if (arg.compare(0, 5, "TNUM=") == 0) {//number of nearest neighbors calculated
			p.num_obj = stoi(arg.substr(5));
		}

		else if (arg.compare(0, 6, "NNREJ=") == 0) {//number of nearest neighbors calculated
			p.nnf_reject_threshold = stof(arg.substr(6));
		}
		else if (arg.compare(0, 5, "NORMF") == 0) {
			p.normal_features = true;
		}
		else if (arg.compare(0, 8, "THREADS=") == 0) {
			p.threads = stof(arg.substr(8));
		}
		else if (arg.compare(0, 5, "CUT_T") == 0) {//
			p.cut_in_template = true;
		}
		else if (arg.compare(0, 6, "RMODE=") == 0) {//
			parse_Rmode(arg.substr(6),p);
		}
		else if (arg.compare(0, 5, "FRAC=") == 0) { // fraction of the model radius used for feature calculation
			p.frac = stof(arg.substr(5));
		}
		else if (arg.compare(0, 8, "DIVFRAC=") == 0) {//the lower it is the more we punish high distortion per match - the precentage of the radius which will lower the score of a match to 1/2
			p.div_frac = stof(arg.substr(8));
		}
		else if (arg.compare(0, 9, "MESHNORMS") == 0) {//calculate mesh normals
			p.mesh_normals = true;;
		}
		else if (arg.compare(0, 4, "MDS=") == 0) {//load an mds of the surface
			p.MDS = arg.substr(4);
		}
		else if (arg.compare(0, 8, "LOAD_DT") == 0) {//load distance matrix from file
			p.load_d_t = true;
		}
		else if (arg.compare(0, 7, "LOAD_DQ") == 0) {//load distance matrix from file
			p.load_d_q = true;
		}
		else if (arg.compare(0, 7, "VIS_SIM") == 0) {//save simliraity function for each mini template
			p.save_similarity_clouds = true;
		}
		else if (arg.compare(0, 9, "SAVEPATCH") == 0) {//save the pathes matching 
 			p.save_feature_patches = true;
		}
		else if (arg.compare(0, 8, "RTHRESH=") == 0) {//fraction of radius to be selected for template candidates
			p.r_thresh = stof(arg.substr(8));
		}
		else if (arg.compare(0, 6, "INDEX=") == 0) {
			p.focal_ind =stoi(arg.substr(6));
		}
		else if (arg.compare(0, 4, "GEO=") == 0) {
			parse_geo(arg.substr(4),p);
		}
		else if (arg.compare(0, 9, "R_SAMPLE=") == 0) {
			p.R_sample = stof(arg.substr(9));
		}
		else if (arg.compare(0, 12, "SAMPLE_PREC=") == 0) {
			p.sample_percent = stoi(arg.substr(12));
		}

	}
}

void create_output_directories(template_match_parms_s& p) {
	mkdir(p.T.path.c_str()); mkdir(p.S.path.c_str());
	p.Rdir = p.T.path + "\\" + p.S.path;
	mkdir(p.Rdir.c_str());
	string tmp_str = "";
	string surface_str = "";
	if ((p.surface_method != NONE) || (p.surface_method != LOAD)) {
		surface_str.append("\\"+p.surface);
		mkdir((p.T.path + "\\" + surface_str).c_str());
		mkdir((p.S.path + "\\" + surface_str).c_str());
		mkdir((p.Rdir + "\\" + surface_str).c_str());
		if (p.surface_method == POISSON) {
			surface_str.append("\\"+ to_string(p.T.s.depth));
			mkdir((p.T.path + "\\" + surface_str).c_str()); p.T.s.out_path = p.T.path + "\\" + surface_str + "\\S.ply";
			mkdir((p.S.path + "\\" + surface_str).c_str()); p.T.s.out_path = p.S.path + "\\" + surface_str + "\\S.ply";
			mkdir((p.Rdir + "\\" + surface_str).c_str());
		}
	}
	tmp_str.append(surface_str);
	tmp_str.append("\\" + p.keypoint); 
	p.T.k.path = p.T.path + tmp_str + "\\key.ply";
	p.S.k.path = p.S.path + tmp_str + "\\key.ply";
	mkdir((p.T.path + tmp_str).c_str());
	mkdir((p.S.path + tmp_str).c_str());
	mkdir((p.Rdir + tmp_str).c_str());
	tmp_str.append("\\"+p.feature);
	mkdir((p.T.path + tmp_str).c_str());
	mkdir((p.S.path + tmp_str).c_str());
	mkdir((p.Rdir + tmp_str).c_str());
	if (p.RMode == DIRECT) {
		p.T.f.path = p.T.path + tmp_str + "\\" + ftos(p.T.f.searchRadius, 3);
		p.S.f.path = p.S.path + tmp_str + "\\" + ftos(p.S.f.searchRadius, 3);
		mkdir(p.T.f.path.c_str());
		mkdir(p.S.f.path.c_str());
		tmp_str.append("\\" + ftos(p.T.f.searchRadius, 3));
		mkdir((p.Rdir + tmp_str).c_str());
		tmp_str.append("\\" + ftos(p.S.f.searchRadius, 3));
		mkdir((p.Rdir + tmp_str).c_str());
	}
	else {
		p.T.f.path = p.T.path + tmp_str + "\\" + p.fractype;
		p.S.f.path = p.S.path + tmp_str + "\\" + p.fractype;
		tmp_str.append("\\" + p.fractype);

		mkdir(p.T.f.path.c_str());
		mkdir(p.S.f.path.c_str());
		mkdir((p.Rdir + tmp_str).c_str());
		tmp_str.append("\\" + ftos(p.frac));

		p.T.f.path = p.T.path + tmp_str + "\\" ;
		p.S.f.path = p.S.path + tmp_str + "\\";
		mkdir(p.T.f.path.c_str());
		mkdir(p.S.f.path.c_str());
		mkdir((p.Rdir + tmp_str).c_str());

	}
	p.T.f.path.append("\\F.pcd");	p.S.f.path.append("\\F.pcd");
	
	p.nnf_dir = p.Rdir + tmp_str + "\\";
	if (p.normal_features) {
		p.nnf_dir.append("NORMF\\");
		mkdir(p.nnf_dir.c_str());
	}
	p.result_dir = p.nnf_dir + p.patch + "\\";
	mkdir(p.result_dir.c_str());
	if (p.pMode != SPHERE) {
		p.result_dir.append(p.mesh + "\\");
		mkdir(p.result_dir.c_str());
	}
	p.result_dir.append(p.sim + "\\");
	mkdir(p.result_dir.c_str());
	if (p.div_frac >0) {
		p.result_dir.append(ftos(p.div_frac, 2) + "\\");
		mkdir(p.result_dir.c_str());
	}
	if (p.r_thresh <100) {
		p.result_dir.append("R_THRESH_"+ftos(p.r_thresh, 2) + "\\");
		mkdir(p.result_dir.c_str());
	}

	p.result_dir.append(p.nn + "\\MAX\\");
	mkdir(p.result_dir.c_str());

}

string format_time_d_h_m_s(time_t elapsed) {

	double seconds = fmod(elapsed, 60);
	string result = ftos(seconds) + "\sS,\s";
	if (elapsed / 60 > 0) {
		int minutes = (int)floor(fmod(elapsed / 60, 60));
		result = to_string(minutes) + "\sM,\s" + result;
	}
	if (elapsed / 3600 > 0) {
		int	hours = (int)floor(fmod(elapsed / 3600, 24));
		result = to_string(hours) + "\sH,\s" + result;
	}
	if (elapsed / (24 * 3600) > 0) {
		int	days = (int)floor(elapsed / (24 * 3600));
		result = to_string(days) + "\sD,\s" + result;
	}
	return result;
}

string TimeLog::printlabel() {
	return label;
}

TimeLog::TimeLog(string label) : label(label) {
	time(&base);
	time(&elapsed);
	logged = false;
}

string TimeLog::LogToCSV() {
	return ftos(difftime(elapsed, base));
}

void TimeLog::SetTime() {
	time(&elapsed);
	logged = true;
}

string TimeLog::format_time_d_h_m_s() {
	float elapsed1 = difftime(elapsed, base);
	double seconds = fmod(elapsed1, 60);
	string result = ftos(seconds) + " S, ";
	if (elapsed1 / 60 > 0) {
		int minutes = (int)floor(fmod(elapsed1 / 60, 60));
		result = to_string(minutes) + " M, " + result;
	}
	if (elapsed1 / 3600 > 0) {
		int	hours = (int)floor(fmod(elapsed1 / 3600, 24));
		result = to_string(hours) + " H, " + result;
	}
	if (elapsed1 / (24 * 3600) > 0) {
		int	days = (int)floor(elapsed1 / (24 * 3600));
		result = to_string(days) + " D, " + result;
	}
	return result;
}

void TimeLog::Reset() {
	time(&base);
	time(&elapsed);
	logged = false;
};


void Time_LogtoCSV(std::vector<TimeLog*> logger, string out_path) {
	fstream stat_file(out_path, std::fstream::out);
	int size = logger.size();
	for (int i = 0; i < size -1; i++) {
		stat_file << logger[i]->printlabel()<<",";
	}
	stat_file << logger[size-1]->printlabel() << endl;
	for (int i = 0; i < size -1; i++) {
		stat_file << logger[i]->LogToCSV() << ",";
	}
	stat_file << logger[size - 1]->LogToCSV() << endl;

}

std::vector<LabeledStat> read_time_log(string filename) {
	std::vector<LabeledStat> result;
	ifstream file(filename);
	std::string labels;
	std::getline(file, labels);
	std::stringstream iss1(labels);
	std::string values;
	std::getline(file, values);
	std::stringstream iss2(values);

	while (true) {
		std::string label;
		std::getline(iss1, label, ',');
		LabeledStat stat;
		stat.Label = label;
		std::string value;
		std::getline(iss2, value, ',');
		stat.stat = stof(value);
		result.push_back(stat);
	}
}

//TODO
int load_center() {
	int center;
	return center;
}

string extract_filename(string s) {
	stringstream iss(s);
	std::string filename;

	while (iss.good())
	{
		getline(iss, filename, '\\');
		// Process SingleLine here
	}
	return filename;
}

bool check_num(std::string const &in) {
	char *end;

	strtol(in.c_str(), &end, 10);
	return !in.empty() && *end == '\0';
}

//////////////////////////////////////////////////////////////////////////////////////////////
void rand_sample_patch(vector<int>& indices, vector<float>& distances, float fraction) {
	vector<int> sample(indices.size());
	iota(sample.begin(), sample.end(), 0);
	random_shuffle(sample.begin(), sample.end());
	int delind = round(fraction*indices.size());
	sample.erase(sample.begin() + delind, sample.end());
	vector<int> temp_indices(sample.size());
	vector<float> temp_distances(sample.size());

	for (int i = 0; i < sample.size(); i++) {
		temp_indices.at(i) = indices.at(sample.at(i));
		temp_distances.at(i) = distances.at(sample.at(i));
	}
	indices = temp_indices;
	distances = temp_distances;
}

void
print4x4Matrix(const Eigen::Matrix4f & matrix)
{
	printf("Rotation matrix :\n");
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
	printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
	printf("Translation vector :\n");
	printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

vector<barycentric_polygon_s> vertex_to_bar(PolygonMeshPtr mesh) {
	cout << "1\n";
	vector<barycentric_polygon_s> result(mesh->cloud.width);

	for (int i = 0; i < mesh->polygons.size(); i++) {
		Vertices p = mesh->polygons.at(i);
		for (int j = 0; j < p.vertices.size(); j++) {
			int v = p.vertices.at(j);
			if (result.at(v).index == -1) {
				result.at(v).index = i+1;
				result.at(v).coor[j] = 1;
			}
		}
	}
	cout << "2\n";

	return result;
}

vector<barycentric_polygon_s> corrs_to_bar_corrs(std::vector<std::vector<dis3::ind_dis_s>> corrs, vector<barycentric_polygon_s> map) {
	vector<barycentric_polygon_s> result(corrs.size());
	for (int i = 0; i < corrs.size(); i++) {
		if (corrs.at(i).size() != 0) {
			result.at(i) = map.at(corrs.at(i).at(0).first);
		}
	}
	return result;
}

void save_bar_corrs(vector<barycentric_polygon_s> result, std::string path) {
	std::fstream outfile(path, std::fstream::out);
	for (int i = 0; i < result.size(); i++) {
		barycentric_polygon_s p = result.at(i);
		outfile << p.index << "\t" << p.coor[0] << "\t" << p.coor[1] << "\t" << p.coor[2] << "\n";
	}
	outfile.close();
}

void save_feature_corrs_to_csv(std::vector<std::vector<dis3::ind_dis_s>> corrs, std::string path, vector<uint32_t> finds) {
	std::fstream outfile(path, std::fstream::out);
	uint32_t rows = finds.size(), cols = corrs.at(finds.at(0)).size();
	/*outfile << "Source ,";
	for (int j = 0; j < cols; j++) {
		outfile << "Ind[" << j << "] ," << "Score[" << j << "] ,";
	}
	outfile << endl;*/
	for (int i = 0; i < rows; i++) {
		uint32_t ind = finds.at(i);
		outfile << to_string(ind);
		for (int j = 0; j <1; j++) {
			outfile << "\t" << corrs.at(ind).at(j).first;// << "," << corrs.at(ind).at(j).second;
		}
		outfile << endl;
	}
	outfile.close();
}

CorrespondencesPtr load_result_corrs_from_csv(const string & path) {
	CorrespondencesPtr result;
	ifstream indata;
	indata.open(path);
	std::string line;
	std::vector<float> values;
	unsigned int rows = 0;
	while (std::getline(indata, line)) {
		std::stringstream lineStream(line);
		std::string cell;
		std::getline(lineStream, cell, ',');
		int count = 0;
		Correspondence corr;
		corr.index_query = stod(cell);
		while (std::getline(lineStream, cell, ',')) {
			corr.index_match = stod(cell);

			std::getline(lineStream, cell, ',');
			corr.distance = stof(cell);
			result->push_back(corr);
		}
	}
	return result;
}

vector<CorrespondencesPtr>  load_feature_corrs_from_csv(const string & path, bool throw_header= false) {
	vector<CorrespondencesPtr> result =
	{ CorrespondencesPtr(new Correspondences()), CorrespondencesPtr(new Correspondences()), CorrespondencesPtr(new Correspondences()), CorrespondencesPtr(new Correspondences()), CorrespondencesPtr(new Correspondences()) };
	ifstream indata;
	indata.open(path);
	std::string line;
	std::vector<float> values;
	unsigned int rows = 0;
	
	if (throw_header) std::getline(indata, line);//header discarding
	while (std::getline(indata, line)) {
		std::stringstream lineStream(line);
		std::string cell;
		std::getline(lineStream, cell, ',');
		int count = 0;
		Correspondence corr;
		corr.index_query = stod(cell);
		while (std::getline(lineStream, cell, ',')) {
			corr.index_match = stod(cell);

			std::getline(lineStream, cell, ',');
			corr.distance=stof(cell);
			result.at(count++)->push_back(corr);
		}
	}
	return result;
}

vector<CorrespondencesPtr>  load_feature_corrs_from_corr(const string & path, bool throw_header = false) {
	vector<CorrespondencesPtr> result =
	{ CorrespondencesPtr(new Correspondences()), CorrespondencesPtr(new Correspondences()), CorrespondencesPtr(new Correspondences()), CorrespondencesPtr(new Correspondences()), CorrespondencesPtr(new Correspondences()) };
	ifstream indata;
	indata.open(path);
	std::string line;
	std::vector<float> values;
	unsigned int rows = 0;

	if (throw_header) std::getline(indata, line);//header discarding
	while (std::getline(indata, line)) {
		std::stringstream lineStream(line);
		std::string cell;
		std::getline(lineStream, cell, '\t');
		int count = 0;
		Correspondence corr;
		corr.index_query = stod(cell);
		while (std::getline(lineStream, cell, '\t')) {
			corr.index_match = stod(cell);

			std::getline(lineStream, cell, '\t');
			corr.distance = stof(cell);
			result.at(count++)->push_back(corr);
		}
	}
	return result;
}
