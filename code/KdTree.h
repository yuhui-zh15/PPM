#ifndef KDTREE_H
#define KDTREE_H
#include <string>
#include "Object.h"
#include "HitPoint.h"

struct KdNode {
	int value, split;
	KdNode *left, *right;
}; 

class KdTree {
private:
	KdNode *_root;
	KdNode *tree;
	int _size;
	int *_index;
	int _cnt;
	static const int K = 3;

public:
	HitPoint *_data;
	~KdTree();
	HitPoint *getData(int &size) const { size = _size; return _data; }
	void load(int size, HitPoint *data);
	void build();
	void insert(const Photon &photon);
	void update();
	void partition(int l, int r, int dim, int k);
	KdNode* build(int l, int r, double *min, double *max);
	void insert(KdNode *node, const Photon &photon);
	void update(KdNode *node);
	void printTree();
};

#endif