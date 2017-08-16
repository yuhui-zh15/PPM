#include "Vec3.h"
#include "KdTree.h"
#include "Object.h"
#include <climits>
#include <iostream>
#include <algorithm>
using namespace std;

void KdTree::load(int size, HitPoint *data) {
	_cnt = 0;
	_size = size;
	_data = data;
	_index = new int[_size];
	tree = new KdNode[_size];
	for (int i = 0; i < _size; ++i) _index[i] = i;
}

void KdTree::partition(int l, int r, int dim, int k) {
	int mid = (l + r) >> 1, tmp = _index[mid];
	int i = l, j = r;
	while (i < j) {
		while (_data[_index[i]].position[dim] < _data[tmp].position[dim]) ++i;
		while (_data[_index[j]].position[dim] > _data[tmp].position[dim]) --j;
		if (i <= j) {
			int t = _index[i]; _index[i] = _index[j]; _index[j] = t;
			++i; --j;
		}
	}
	if (l < j && l <= k && k <= j) partition(l, j, dim, k);
	if (i < r && i <= k && k <= r) partition(i, r, dim, k);
}

KdNode* KdTree::build(int l, int r, double *min, double *max)
{
	if (r <= l) return NULL;
	double tmp = -1; int split;
	for (int i = 0; i < K; ++i) {
		if (max[i] - min[i] > tmp) {
			tmp = max[i] - min[i];
			split = i;
		}
	}
	int mid = (l + r) >> 1;
	partition(l, r - 1, split, mid);
	tree[_cnt].value = _index[mid];
	tree[_cnt].split = split;
	KdNode *node = &tree[_cnt++];

	tmp = max[split];
	max[split] = _data[_index[mid]].position[split];
	node->left = build(l, mid, min, max);
	max[split] = tmp;

	tmp = min[split];
	min[split] = _data[_index[mid]].position[split];
	node->right = build(mid + 1, r, min, max);
	min[split] = tmp;

	_data[node->value].maxRadius2 = _data[node->value].radius2;
	if (node->left && _data[node->left->value].maxRadius2 > _data[node->value].maxRadius2)
		_data[node->value].maxRadius2 = _data[node->left->value].maxRadius2;

	if (node->right && _data[node->right->value].maxRadius2 > _data[node->value].maxRadius2)
		_data[node->value].maxRadius2 = _data[node->right->value].maxRadius2;

	return node;
}

void KdTree::build()
{
	_cnt = 0;
	double *min = new double[K], *max = new double[K];
	for (int i = 0; i < K; ++i) min[i] = LONG_MAX, max[i] = LONG_MIN;
	for (int i = 0; i < _size; ++i) {
		Vec3 tmp = _data[i].position;;
		for (int j = 0; j < K; ++j) {
			if (_data[i].position[j] < min[j]) min[j] = _data[i].position[j];
			if (_data[i].position[j] > max[j]) max[j] = _data[i].position[j];
		}
	}
	_root = build(0, _size, min, max);
}

void KdTree::insert(KdNode *node, const Photon &photon)
{
	if (node == NULL) return;

	int pos = node->value;
	if (dot(_data[pos].position - photon.position, _data[pos].position - photon.position) < _data[pos].radius2)
		if (photon.object == _data[pos].object) {
			_data[pos].M += 1;
			double diffuse = _data[pos].object->getDiffuse();
			_data[pos].totalFlux += photon.color * diffuse;
		}

	int split = node->split;
	KdNode *another;
	if (photon.position[split] < _data[pos].position[split]) {
		another = node->right;
		insert(node->left, photon);
	}
	else {
		another = node->left;
		insert(node->right, photon);
	}
	if ((another) && (_data[pos].position[split] - photon.position[split]) * (_data[pos].position[split] - photon.position[split]) < _data[another->value].maxRadius2 + EPSI)
		insert(another, photon);
}

void KdTree::insert(const Photon &photon) {
	insert(_root, photon);
}

void KdTree::update(KdNode *node) {
	if (node->left) update(node->left);
	if (node->right) update(node->right);
	_data[node->value].maxRadius2 = _data[node->value].radius2;
	if (node->left && _data[node->left->value].maxRadius2 > _data[node->value].maxRadius2)
		_data[node->value].maxRadius2 = _data[node->left->value].maxRadius2;

	if (node->right && _data[node->right->value].maxRadius2 > _data[node->value].maxRadius2)
		_data[node->value].maxRadius2 = _data[node->right->value].maxRadius2;
}

void KdTree::update() {
	update(_root);
}

void KdTree::printTree() {
	for (int i = 0; i < _size; ++i) {
		int pos = tree[i].value;
		KdNode *lc = tree[i].left, *rc = tree[i].right;
		std::cout << "Number: " << i << " Split: " << tree[i].split << std::endl;
		std::cout << "Postion: " << _data[pos].position[0] << ' ' << _data[pos].position[1] << ' ' << _data[pos].position[2] << std::endl;
		std::cout << "Radius: " << _data[pos].radius2 << ' ' << _data[pos].maxRadius2 << std::endl;
		if (lc)
			std::cout << "Left child: " << _data[lc->value].position[0] << ' ' << _data[lc->value].position[1] << ' ' << _data[lc->value].position[2] << std::endl;
		else 
			std::cout << "Left child: NULL" << std::endl;
		if (rc)
			std::cout << "Right child: " << _data[rc->value].position[0] << ' ' << _data[rc->value].position[1] << ' ' << _data[rc->value].position[2] << std::endl;
		else 
			std::cout << "Right child: NULL" << std::endl;
		std::cout << std::endl;
	}
	std::cout << "Finished!" << std::endl;
}

KdTree::~KdTree() {
	delete[] _root;
	delete[] _index;
	delete[] tree;
}