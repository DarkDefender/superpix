#include "superpix_helper_func.h"

#include <iostream>

//Nanosvg deps begin
#include <stdio.h>
#include <string.h>
#include <math.h>
#define NANOSVG_IMPLEMENTATION	// Expands implementation
#include "nanosvg/src/nanosvg.h"
//Nanosvg deps end

#include "write_png.h"

using namespace std;

void print_bez(float* p){
	for (int i = 0; i < 4; i++){
		cout << "P" << i << ":\n";
		cout << "X: " << p[i*2] << " ";
		cout << "Y: " << p[(i*2)+1] << " ";
		cout << endl;
	}
}

void debug_print_paths(string svg_file){
	// Load
	struct NSVGimage* image;
	image = nsvgParseFromFile(svg_file.c_str(), "px", 96);
	printf("size: %f x %f\n", image->width, image->height);
	// Use...
	for (auto shape = image->shapes; shape != NULL; shape = shape->next) {
		for (auto path = shape->paths; path != NULL; path = path->next) {
			for (int i = 0; i < path->npts-1; i += 3) {
				float* p = &path->pts[i*2];
				print_bez(p);
			}
		}
	}

	// Delete
	nsvgDelete(image);
}

vector<Path> read_svg_paths(const char *svg_file, int &w, int &h){
	vector<Path> read_paths;

	bool poly_path = false;

	// Load
	struct NSVGimage* image;
	image = nsvgParseFromFile(svg_file, "px", 96);
	cout << "Reading: " << svg_file << endl;

	if (image == NULL) {
		cout << "Couldn't open: " << svg_file << endl;
		return read_paths;
	}

	// Use...
	for (auto shape = image->shapes; shape != NULL; shape = shape->next) {
		for (auto path = shape->paths; path != NULL; path = path->next) {
			Path new_path;
			for (int i = 0; i < path->npts-1; i += 3) {
				float* p = &path->pts[i*2];
				my_point p0, p1, p2, p3;
				p0.x = p[0]; p0.y = p[1];
				p1.x = p[2]; p1.y = p[3];
				p2.x = p[4]; p2.y = p[5];
				p3.x = p[6]; p3.y = p[7];

				if(poly_path) {
					Curve cur{p0,p0,p3,p3};
					new_path.add_curve(cur);
				} else {
					Curve cur{p0,p1,p2,p3};
					new_path.add_curve(cur);
				}
			}
			read_paths.push_back(new_path);
		}
	}

	w = image->width;
	h = image->height;

	// Delete
	nsvgDelete(image);

	return read_paths;
}

int main(int argc, const char * argv[]){

    if (argc != 3){
		cout << "Not enough input arguments given!" << endl;
		cout << "You need to specify input svg file path and output png path:" << endl;
		cout << "Example: ./super_pix ../test.svg output.png" << endl;
		return 0;
	}

	vector<Path> paths;
    vector<vector<my_point>> points;
    int w,h;

    paths = read_svg_paths(argv[1], w, h);

	if (paths.size() == 0) {
		cout << "No paths read! Exiting..." << endl;
		return 0;
	}

	//cout << "path vec:\n";
	//cout << paths.size() << endl;
	//cout << paths[0].get_num_segments() << endl;

	for(size_t i = 0; i < paths.size(); i++){
		points.push_back( superpix_helper_func(paths[i]) );
	}

	//cout << "points vec:\n";
	//cout << points.size() << endl;
	//cout << points[0].size() << endl;

	//for (auto p : points[0]){
	//	cout << p << endl;
	//}

	test_png(argv[2], points, w, h);

	return 0;
}
