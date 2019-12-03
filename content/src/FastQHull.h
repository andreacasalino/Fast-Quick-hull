/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
*
* report any bug to andrecasa91@gmail.com.
 **/

#pragma once
#ifndef __FAST_QUICK_HULL_H__
#define __FAST_QUICK_HULL_H__

#include <list>
/* in case OpenMP (https://en.wikipedia.org/wiki/OpenMP) is enabled, Fast_QHull is automatically built in the parallel computaion mode, using a 
number of threads equal to the number of cores available. See also Fast_QHull.pdf
*/
#if defined(_OPENMP)
#include <omp.h>
#endif

#define USE_CRASHING_ABORT
#define GEOMETRIC_TOLLERANCE (float)1e-3


/** \brief The solver to use for computing the convex hull of a point cloud.
* \details V can be any type representing the 3D Cartesian coordinates of a point in the space.
typename V must have at least:

-> A constructor accepting the 3 floats representing the coordinates	V::V(const float& X, const float& Y, const float& Z);

-> Three getters returning each coordinate, defined in this way (with the obvious meaning of notation):
		-> const float& V::x() const;
		-> const float& V::y() const;
		-> const float& V::z() const;
*/
template<typename V>
class Fast_QHull {
public:
	Fast_QHull() { this->Vertices = NULL;  };

	/** \brief Use this for computing a new convex hull.
	* \details The passed cloud is analyzed and the incidences (see Fast_QHull.pdf) of the convex hull are internally stored. 
	For accessing the incidences use Fast_QHull::Get_Incidences, while the normals of the facet can be obtained by invoking 
	Fast_QHull::Get_Normals (the same order of facets is assumed for the incidences and the normals). 
	* @param[in] new_set_of_vertices The list of vertices characterizing the point cloud to consider for the convex hull computation (Is the cloud C in the Fast_QHull.pdf)
	* @param[in] max_iterations The maximum number of iterations to consider for updating the convex hull (see Fast_QHull.pdf). Infinity is assumed as default, 
	passing 0: the algorithm stops only after the convex hull is completely obtained
	*/
	void Compute_new_Convex_Hull(const std::list<V>* new_set_of_vertices, const size_t& max_iterations = 0);

	/** \brief Returns the incidences (see the documentation) of the convex hull triangulation.
	* \details The incidences are meant here as the positions of the vertex in the cloud passed in Fast_QHull::Compute_new_Convex_Hull. See also Fast_QHull.pdf
	The incidences are computed and internally stored after callling Fast_QHull::Compute_new_Convex_Hull. Therefore,
	when this function is invoked, the incidences of the convex hull of the last passed cloud are returned. In case the convex hull computation
	was not successfull, an empty list is returned.
	* @param[out] indexes The list of incidences characterizing the convex hull
	*/
	void Get_Incidences(std::list<std::list<size_t>>* indexes) const;

	/** \brief Similar to Fast_QHull::Get_Incidences(std::list<std::list<size_t>>* indexes).
	* \details Here the incidences are stored in a vector of positions, which is allocated internally with a malloc operation and then returned. Remember to
	deallocate the buffer of positions after having use it. In case the cinvex hull computation was not successfull, a NULL result is returned
	* @param[out] return The list of incidences characterizing the convex hull
	* @param[out] numb_of_facets the number of facets
	*/
	size_t* Get_Incidences(size_t* numb_of_facets) const;

	/** \brief Returns the incidences of the convex hull triangulation.
	* \details The incidences are computed and internally stored after callling Fast_QHull::Compute_new_Convex_Hull. Therefore,
	when this function is invoked, the incidences of the convex hull of the last passed cloud are returned.  In case the convex hull computation
	was not successfull, an empty list is returned
	* @param[out] indexes The list of incidences characterizing the convex hull
	*/
	void Get_Normals(std::list<V>* normals) const;
private:
	struct Facet {
		bool					bVisible; //only for update Hull

		const V*				A; //pointer of the vertex A
		const V*				B; //pointer of the vertex B
		const V*				C; //pointer of the vertex C

		size_t					pos_A; //pos in the list of vertices of point A
		size_t					pos_B; //pos in the list of vertices of point B
		size_t					pos_C; //pos in the list of vertices of point C

		float						N[3]; //outer normal 

		Facet*				Neighbour[3]; // AB, BC, CA

		const V*				Furthest_vertex; //if different form NULL this facet can be considered for further expansion of the hull
		size_t					Furthest_Pos;
		float						Dist_Furthest_vertex;
	};

	void __Compute_new_Convex_Hull_Master_thread(const size_t& max_iterations);
	void __init_thetraedron();
	void __Update_Hull(Facet* facet_to_expand);
	void __Append_facet(const V* vertexA, const V* vertexB, const V* vertexC, const size_t& pos_A, const size_t& pos_B, const size_t& pos_C );
	void __Recompute_Normal(Facet* facet);
	void __Recompute_farthest(Facet* involved_face);
#if defined(_OPENMP)
	void __Recompute_farthest_thread(const int& th_id);
#endif
	void __Replace(Facet* involved_facet, Facet* old_neigh, Facet* new_neigh);


	void ____distance_to_point(float* result, const V* vertex, const V* A);
	void ____distance_to_line(float* result, const V* vertex, const V* A, const V* B);

// data
	const std::list<V>*				Vertices;
	std::list<Facet>					Facets;
	float											Mid_point[3];
#if defined(_OPENMP)
	struct Farthest_info {
		const V*								local_Furthest_vertex;
		size_t									local_Furthest_Pos;
		float										local_Dist_Furthest_vertex;
	};
	int											Threads_number;
	bool										Threads_life;
	const Facet*							Thread_Involved_facet;
	std::list<Farthest_info>		Farthest_found;
	std::list<bool>						Thrad_life_local_copy;
#endif
};


///////////////////////////////////////////////////////
//							Implementations									 //
///////////////////////////////////////////////////////


template<typename V>
void Fast_QHull<V>::____distance_to_point(float* result, const V* vertex, const V* A) {

	*result = (vertex->x() - A->x()) * (vertex->x() - A->x());
	*result += (vertex->y() - A->y()) * (vertex->y() - A->y());
	*result += (vertex->z() - A->z()) * (vertex->z() - A->z());

};

template<typename V>
void Fast_QHull<V>::____distance_to_line(float* result, const V* vertex, const V* A, const V* B) {

	float V1[3];
	V1[0] = A->x() - vertex->x();
	V1[1] = A->y() - vertex->y();
	V1[2] = A->z() - vertex->z();
	float V2[3];
	V2[0] = B->x() - A->x();
	V2[1] = B->y() - A->y();
	V2[2] = B->z() - A->z();
	float s_opt =  -( V1[0]*V2[0] + V1[1] * V2[1] + V1[2] * V2[2]  )/(V2[0] * V2[0] + V2[1] * V2[1] + V2[2] * V2[2]);

	*result = 2.f * s_opt *( V1[0] * V2[0] + V1[1] * V2[1] + V1[2] * V2[2] );
	*result += s_opt*s_opt*(V2[0] * V2[0] + V2[1] * V2[1] + V2[2] * V2[2]);
	*result += (V1[0] * V1[0] + V1[1] * V1[1] + V1[2] * V1[2]);

}

template<typename V>
void Fast_QHull<V>::__Recompute_farthest(Facet* involved_face) {

#if defined(_OPENMP)

	this->Thread_Involved_facet = involved_face;
	this->__Recompute_farthest_thread(0);
	//do reduction
	auto it_far = this->Farthest_found.begin();
	Farthest_info* best = &(*it_far);
	it_far++;
	for (it_far; it_far != this->Farthest_found.end(); it_far++) {
		if (it_far->local_Furthest_vertex != NULL) {
			if (it_far->local_Dist_Furthest_vertex > best->local_Dist_Furthest_vertex)
				best = &(*it_far);
		}
	}
	involved_face->Dist_Furthest_vertex = best->local_Dist_Furthest_vertex;
	involved_face->Furthest_vertex = best->local_Furthest_vertex;
	involved_face->Furthest_Pos = best->local_Furthest_Pos;

#else

	auto it = this->Vertices->begin();
	involved_face->Furthest_vertex = &(*it);
	involved_face->Furthest_Pos = 0;
	involved_face->Dist_Furthest_vertex = (it->x() - involved_face->A->x()) * involved_face->N[0] + (it->y() - involved_face->A->y()) * involved_face->N[1] + (it->z() - involved_face->A->z()) * involved_face->N[2];
	it++;
	size_t k = 1;
	float distance;
	for (it; it != Vertices->end(); it++) {
		distance = (it->x() - involved_face->A->x()) * involved_face->N[0] + (it->y() - involved_face->A->y()) * involved_face->N[1] + (it->z() - involved_face->A->z()) * involved_face->N[2];
		if (distance > involved_face->Dist_Furthest_vertex) {
			involved_face->Dist_Furthest_vertex = distance;
			involved_face->Furthest_vertex = &(*it);
			involved_face->Furthest_Pos = k;
		}
		k++;
	}

#endif

}

#if defined(_OPENMP)
template<typename V>
void Fast_QHull<V>::__Recompute_farthest_thread(const int& th_id) {

#pragma omp barrier
	if (!this->Threads_life) {
		auto it_life = this->Thrad_life_local_copy.begin();
		advance(it_life, th_id);
		*it_life = false;
		return;
	}

	auto it_far = this->Farthest_found.begin();
	advance(it_far, th_id);
	it_far->local_Furthest_vertex = NULL;

	if ((th_id + 1) > this->Vertices->size()) 
		return;

	auto it = this->Vertices->begin();
	advance(it, th_id);

	it_far->local_Furthest_vertex = &(*it);
	it_far->local_Furthest_Pos = th_id;
	it_far->local_Dist_Furthest_vertex = (it->x() - this->Thread_Involved_facet->A->x()) * this->Thread_Involved_facet->N[0] + (it->y() - this->Thread_Involved_facet->A->y()) * this->Thread_Involved_facet->N[1] + (it->z() - this->Thread_Involved_facet->A->z()) * this->Thread_Involved_facet->N[2];

	size_t k = th_id + this->Threads_number;
	size_t K_max = this->Vertices->size();
	if (k >= K_max) return;
	advance(it, this->Threads_number);
	float distance;
	while (it != Vertices->end()) {
		distance = (it->x() - this->Thread_Involved_facet->A->x()) * this->Thread_Involved_facet->N[0] + (it->y() - this->Thread_Involved_facet->A->y()) * this->Thread_Involved_facet->N[1] + (it->z() - this->Thread_Involved_facet->A->z()) * this->Thread_Involved_facet->N[2];
		if (distance > it_far->local_Dist_Furthest_vertex) {
			it_far->local_Dist_Furthest_vertex = distance;
			it_far->local_Furthest_vertex = &(*it);
			it_far->local_Furthest_Pos = k;
		}
		k += this->Threads_number;
		if (k >= K_max) break;
		advance(it, this->Threads_number);
	}
#pragma omp barrier

}
#endif

template<typename V>
void Fast_QHull<V>::__init_thetraedron() {

	//Start with building an initial non zero volume thetraedron. 
	const V* far_1, *far_2, *far_3, *far_4;
	size_t pos_1, pos_2, pos_3, pos_4;

	float dist_max, dist;
	size_t p;

	//first point is farthest along x axis
	auto it_V = this->Vertices->begin();
	dist_max = it_V->x();
	far_1 = &(*it_V);
	pos_1 = 0;
	it_V++;
	p = 1;
	for (it_V; it_V != this->Vertices->end(); it_V++) {
		if (it_V->x()  > dist_max) {
			dist_max = it_V->x();
			far_1 = &(*it_V);
			pos_1 = p;
		}
		p++;
	}

	//the second point considered will be the farthest point from the point P1
	it_V = this->Vertices->begin();
	this->____distance_to_point(&dist_max, &(*it_V), far_1);
	far_2 = &(*it_V);
	pos_2 = 0;
	it_V++;
	p = 1;
	for (it_V; it_V != this->Vertices->end(); it_V++) {
		this->____distance_to_point(&dist, &(*it_V), far_1);
		if (dist > dist_max) {
			dist_max = dist;
			far_2 = &(*it_V);
			pos_2 = p;
		}
		p++;
	}

	//the third point considered will be the farthest point of the Cloud from the line P1P2
	it_V = this->Vertices->begin();
	this->____distance_to_line(&dist_max, &(*it_V),  far_2, far_1);
	far_3 = &(*it_V);
	pos_3 = 0;
	it_V++;
	p = 1;
	for (it_V; it_V != this->Vertices->end(); it_V++) {
		this->____distance_to_line(&dist, &(*it_V), far_2, far_1);
		if (dist > dist_max) {
			dist_max = dist;
			far_3 = &(*it_V);
			pos_3 = p;
		}
		p++;
	}

	float V1[3];
	V1[0] = far_1->x() - far_3->x();
	V1[1] = far_1->y() - far_3->y();
	V1[2] = far_1->z() - far_3->z();
	float V2[3];
	V2[0] = far_2->x() - far_3->x();
	V2[1] = far_2->y() - far_3->y();
	V2[2] = far_2->z() - far_3->z();
	float N[3];
	N[0] = V1[1] * V2[2] - V1[2] * V2[1];
	N[1] = V1[2] * V2[0] - V1[0] * V2[2];
	N[2] = V1[0] * V2[1] - V1[1] * V2[0];

	//the fourth point will be the farthest point of the Cloud, from the plane P1P2P3
	it_V = this->Vertices->begin();
 	dist_max = abs(N[0] * (it_V->x() - far_1->x()) + N[1] * (it_V->y() - far_1->y()) + N[2] * (it_V->z() - far_1->z()));
	far_4 = &(*it_V);
	pos_4 = 0;
	it_V++;
	p = 1;
	for (it_V; it_V != this->Vertices->end(); it_V++) {
		dist = abs(N[0] * (it_V->x() - far_1->x()) + N[1] * (it_V->y() - far_1->y()) + N[2] * (it_V->z() - far_1->z()));
		if (dist > dist_max) {
			dist_max = dist;
			far_4 = &(*it_V);
			pos_4 = p;
		}
		p++;
	}

	//check the thetraedron has a non zero volume
	float delta_1[3];
	delta_1[0] = far_4->x() - far_1->x();
	delta_1[1] = far_4->y() - far_1->y();
	delta_1[2] = far_4->z() - far_1->z();
	float delta_2[3];
	delta_2[0] = far_4->x() - far_2->x();
	delta_2[1] = far_4->y() - far_2->y();
	delta_2[2] = far_4->z() - far_2->z();
	float delta_3[3];
	delta_3[0] = far_4->x() - far_3->x();
	delta_3[1] = far_4->y() - far_3->y();
	delta_3[2] = far_4->z() - far_3->z();

	float cross_1_2[3];
	cross_1_2[0] = delta_1[1] * delta_2[2] - delta_1[2] * delta_2[1];
	cross_1_2[1] = delta_1[2] * delta_2[0] - delta_1[0] * delta_2[2];
	cross_1_2[2] = delta_1[0] * delta_2[1] - delta_1[1] * delta_2[0];
	float volume = cross_1_2[0] * delta_3[0] + cross_1_2[1] * delta_3[1] + cross_1_2[2] * delta_3[2];
	if (abs(volume) < GEOMETRIC_TOLLERANCE) {
		this->Facets.clear();
		this->Vertices = NULL;
#ifdef USE_CRASHING_ABORT
		abort();
#else
		return;
#endif
	}

	//computation of the midpoint of the thetraedron
	this->Mid_point[0] = 0.25f * (far_1->x() + far_2->x() + far_3->x() + far_4->x());
	this->Mid_point[1] = 0.25f * (far_1->y() + far_2->y() + far_3->y() + far_4->y());
	this->Mid_point[2] = 0.25f * (far_1->z() + far_2->z() + far_3->z() + far_4->z());

	//build the tethraedron
	//ABC->0; ABD->1; ACD->2; BCD->3
	this->__Append_facet(far_1, far_2, far_3, pos_1, pos_2, pos_3);
	this->__Append_facet(far_1, far_2, far_4, pos_1, pos_2, pos_4);
	this->__Append_facet(far_1, far_3, far_4, pos_1, pos_3, pos_4);
	this->__Append_facet(far_2, far_3, far_4, pos_2, pos_3, pos_4);

	auto itFace_ABC = this->Facets.begin();
	auto itFace_ABD = itFace_ABC; itFace_ABD++;
	auto itFace_ACD = itFace_ABD; itFace_ACD++;
	auto itFace_BCD = itFace_ACD; itFace_BCD++;

	//ABC
	itFace_ABC->Neighbour[0] = &(*itFace_ABD);
	itFace_ABC->Neighbour[1] = &(*itFace_BCD);
	itFace_ABC->Neighbour[2] = &(*itFace_ACD);

	//ABD
	itFace_ABD->Neighbour[0] = &(*itFace_ABC);
	itFace_ABD->Neighbour[1] = &(*itFace_BCD);
	itFace_ABD->Neighbour[2] = &(*itFace_ACD);

	//ACD
	itFace_ACD->Neighbour[0] = &(*itFace_ABC);
	itFace_ACD->Neighbour[1] = &(*itFace_BCD);
	itFace_ACD->Neighbour[2] = &(*itFace_ABD);

	//BCD
	itFace_BCD->Neighbour[0] = &(*itFace_ABC);
	itFace_BCD->Neighbour[1] = &(*itFace_ACD);
	itFace_BCD->Neighbour[2] = &(*itFace_ABD);

}

template<typename V>
void Fast_QHull<V>::__Recompute_Normal(Facet* facet) {

	float delta1[3], delta2[3];
	delta1[0] = facet->A->x() - facet->C->x();
	delta1[1] = facet->A->y() - facet->C->y();
	delta1[2] = facet->A->z() - facet->C->z();

	delta2[0] = facet->B->x() - facet->C->x();
	delta2[1] = facet->B->y() - facet->C->y();
	delta2[2] = facet->B->z() - facet->C->z();

	facet->N[0] = delta1[1] * delta2[2] - delta1[2] * delta2[1];
	facet->N[1] = delta1[2] * delta2[0] - delta1[0] * delta2[2];
	facet->N[2] = delta1[0] * delta2[1] - delta1[1] * delta2[0];

	float dot_normal;
	delta1[0] = this->Mid_point[0] - facet->A->x();
	delta1[1] = this->Mid_point[1] - facet->A->y();
	delta1[2] = this->Mid_point[2] - facet->A->z();
	dot_normal = delta1[0] * facet->N[0] + delta1[1] * facet->N[1] + delta1[2] * facet->N[2];
	if (dot_normal >= 0.f) {
		facet->N[0] = -facet->N[0];
		facet->N[1] = -facet->N[1];
		facet->N[2] = -facet->N[2];
	}

	//normalize
	dot_normal = sqrtf(facet->N[0] * facet->N[0] + facet->N[1] * facet->N[1] + facet->N[2] * facet->N[2]);
	if (dot_normal < 1e-7) {
		facet->N[0] = (float)1e6 * facet->N[0];
		facet->N[1] = (float)1e6  * facet->N[1];
		facet->N[2] = (float)1e6  * facet->N[2];
	}
	else {
		dot_normal = 1.f / dot_normal;
		facet->N[0] = dot_normal * facet->N[0];
		facet->N[1] = dot_normal * facet->N[1];
		facet->N[2] = dot_normal * facet->N[2];
	}

	this->__Recompute_farthest(facet);
	if (facet->Dist_Furthest_vertex <= GEOMETRIC_TOLLERANCE)
		facet->Furthest_vertex = NULL;

	if((facet->Furthest_vertex == facet->A )||(facet->Furthest_vertex == facet->B)||(facet->Furthest_vertex == facet->C))
		facet->Furthest_vertex = NULL;

}

template<typename V>
void Fast_QHull<V>::__Append_facet(const V* vertexA, const V* vertexB, const V* vertexC, const size_t& pos_A, const size_t& pos_B, const size_t& pos_C) {

	Facet new_face;
	new_face.bVisible = false;
	new_face.A = vertexA;
	new_face.B = vertexB;
	new_face.C = vertexC;

	new_face.pos_A = pos_A;
	new_face.pos_B = pos_B;
	new_face.pos_C = pos_C;
	
	//normal computation
	this->__Recompute_Normal(&new_face);

	this->Facets.push_back(new_face);

}

template<typename V>
void Fast_QHull<V>::__Replace(Facet* involved_facet, Facet* old_neigh, Facet* new_neigh) {
	if (old_neigh == involved_facet->Neighbour[0]) involved_facet->Neighbour[0] = new_neigh;
	else {
		if (old_neigh == involved_facet->Neighbour[1]) involved_facet->Neighbour[1] = new_neigh;
		else involved_facet->Neighbour[2] = new_neigh;
	}
};

template<typename V>
void Fast_QHull<V>::__Update_Hull(Facet* facet_to_expand) {

	facet_to_expand->bVisible = true;
	const V* Point = facet_to_expand->Furthest_vertex;
	size_t Point_pos = facet_to_expand->Furthest_Pos;

	//find the group of visible faces
	std::list<Facet*> Visible_group;
	Visible_group.push_back(facet_to_expand);
	auto itN = Visible_group.begin();
	size_t kNeigh = 0, k;
	float distance;
	while (kNeigh < Visible_group.size()) {
		for (k = 0; k < 3; k++) {
			if (!(*itN)->Neighbour[k]->bVisible) { //this neighbour facet is not already present in Visible_Group
				distance = (*itN)->Neighbour[k]->N[0] * (Point->x() - (*itN)->Neighbour[k]->A->x());
				distance += (*itN)->Neighbour[k]->N[1] * (Point->y() - (*itN)->Neighbour[k]->A->y());
				distance += (*itN)->Neighbour[k]->N[2] * (Point->z() - (*itN)->Neighbour[k]->A->z());
				if (distance > GEOMETRIC_TOLLERANCE) {
					(*itN)->Neighbour[k]->bVisible = true;
					Visible_group.push_back((*itN)->Neighbour[k]);
				}
			}
		}
		kNeigh++;
		itN++;
	}
	size_t dim_visible = Visible_group.size();

	//Update Hull. Build the cone of new facets
	const V* A, * B, * C;
	Facet* AB, * BC, * CA;
	size_t pos_vertex_old[3];
	itN = Visible_group.begin();
	size_t kboard;
	bool delete_face = false;
	for (k = 0; k < dim_visible; k++) {
		kboard = 0;
		A = (*itN)->A; B = (*itN)->B; C = (*itN)->C;
		AB = (*itN)->Neighbour[0]; BC = (*itN)->Neighbour[1]; CA = (*itN)->Neighbour[2];
		pos_vertex_old[0] = (*itN)->pos_A; pos_vertex_old[1] = (*itN)->pos_B; pos_vertex_old[2] = (*itN)->pos_C;

		//AB
		if (!AB->bVisible) { //edge AB is part of the board of the cone
			(*itN)->C = Point;
			(*itN)->pos_C = Point_pos;
			this->__Recompute_Normal(*itN);
			kboard++;
		}
		//BC
		if (!BC->bVisible) { //edge BC is part of the board of the cone
			if (kboard == 0) {
				(*itN)->A = B;
				(*itN)->B = C;
				(*itN)->C = Point;
				(*itN)->pos_A = pos_vertex_old[1];
				(*itN)->pos_B = pos_vertex_old[2];
				(*itN)->pos_C = Point_pos;
				this->__Recompute_Normal(*itN);
				(*itN)->Neighbour[0] = BC;
			}
			else {
				this->__Append_facet(B, C, Point, pos_vertex_old[1], pos_vertex_old[2], Point_pos);
				this->Facets.back().bVisible = true;
				Visible_group.push_back(&this->Facets.back());
				Visible_group.back()->Neighbour[0] = BC;
				this->__Replace(BC, *itN, Visible_group.back());
			}
			kboard++;
		}
		//CA
		if (!CA->bVisible) { //edge CA is part of the board of the cone
			if (kboard == 0) {
				(*itN)->A = A;
				(*itN)->B = C;
				(*itN)->C = Point;
				(*itN)->pos_A = pos_vertex_old[0];
				(*itN)->pos_B = pos_vertex_old[2];
				(*itN)->pos_C = Point_pos;
				this->__Recompute_Normal(*itN);
				(*itN)->Neighbour[0] = CA;
			}
			else {
				this->__Append_facet(A, C, Point, pos_vertex_old[0], pos_vertex_old[2], Point_pos);
				this->Facets.back().bVisible = true;
				Visible_group.push_back(&this->Facets.back());
				Visible_group.back()->Neighbour[0] = CA;
				this->__Replace(CA, *itN, Visible_group.back());
			}
			kboard++;
		}

		if (kboard == 0) {
			delete_face = true;
			(*itN)->Neighbour[0] = NULL;
			itN = Visible_group.erase(itN);
		}
		else itN++;
	}

	if (delete_face) { //remove facet for which Neighbour[0]=NULL
		auto itF = this->Facets.begin();
		while (itF != this->Facets.end()) {
			if (itF->Neighbour[0] == NULL)
				itF = this->Facets.erase(itF);
			else
				itF++;
		}

	}


	//delete old neighbour info for the visible group
	for (itN = Visible_group.begin(); itN != Visible_group.end(); itN++) {
		(*itN)->Neighbour[1] = NULL; (*itN)->Neighbour[2] = NULL;
	}

	//update neighbour info for the visible group 
	auto itN2 = itN;
	for (itN = Visible_group.begin(); itN != Visible_group.end(); itN++) {
		if ((*itN)->Neighbour[1] == NULL) { //find neighbour of edge BC
			for (itN2 = Visible_group.begin(); itN2 != Visible_group.end(); itN2++) {
				if (*itN2 != *itN) {
					if ((*itN2)->A == (*itN)->B) {
						(*itN)->Neighbour[1] = *itN2;
						(*itN2)->Neighbour[2] = *itN;
						break;
					}

					if ((*itN2)->B == (*itN)->B) {
						(*itN)->Neighbour[1] = *itN2;
						(*itN2)->Neighbour[1] = *itN;
						break;
					}
				}
			}
		}

		if ((*itN)->Neighbour[2] == NULL) { //find neighbour of edge CA
			for (itN2 = Visible_group.begin(); itN2 != Visible_group.end(); itN2++) {
				if (*itN2 != *itN) {
					if ((*itN2)->A == (*itN)->A) {
						(*itN)->Neighbour[2] = *itN2;
						(*itN2)->Neighbour[2] = *itN;
						break;
					}

					if ((*itN2)->B == (*itN)->A) {
						(*itN)->Neighbour[2] = *itN2;
						(*itN2)->Neighbour[1] = *itN;
						break;
					}
				}
			}
		}
	}

	for (itN = Visible_group.begin(); itN != Visible_group.end(); itN++) 
		(*itN)->bVisible = false;

}

template<typename V>
void Fast_QHull<V>::__Compute_new_Convex_Hull_Master_thread(const size_t& max_iterations) {

	this->__init_thetraedron();
	if (this->Facets.empty()) return;

	size_t Iterations_to_adopt, k_iter = 4;
	if (max_iterations == 0)
		Iterations_to_adopt = 0;
	else if (max_iterations <= 4)
		return;
	else
		Iterations_to_adopt = max_iterations - 4;

	//expand the initial thetraedron. Stop when there are no more facet expandable
	Facet* f_to_expand;
	auto it_facet = this->Facets.begin();
	while (true) {
		f_to_expand = NULL;
		it_facet = this->Facets.begin();
		for (it_facet; it_facet != this->Facets.end(); it_facet++) {
			if (it_facet->Furthest_vertex != NULL) {
				f_to_expand = &(*it_facet);
				break;
			}
		}
		if (f_to_expand == NULL) break;

		for (it_facet; it_facet != this->Facets.end(); it_facet++) {
			if (it_facet->Furthest_vertex != NULL) {
				if (it_facet->Dist_Furthest_vertex > f_to_expand->Dist_Furthest_vertex)
					f_to_expand = &(*it_facet);
			}
		}
		this->__Update_Hull(f_to_expand);

		k_iter++;
		if (k_iter == Iterations_to_adopt) break;
	}

}

template<typename V>
void Fast_QHull<V>::Compute_new_Convex_Hull(const std::list<V>* new_set_of_vertices, const size_t& max_iterations) {

	this->Facets.clear();
	this->Vertices = NULL;

	if (new_set_of_vertices->size()  < 4) {
#ifdef		USE_CRASHING_ABORT
		abort();
#else
		return;
#endif
	}
	
	this->Vertices = new_set_of_vertices;

#if defined(_OPENMP)
	this->Threads_number = omp_get_num_procs();
	this->Threads_life = true;
	this->Farthest_found.clear();
	this->Thrad_life_local_copy.clear();
	for (size_t k = 0; k < this->Threads_number; k++) {
		this->Farthest_found.push_back(Farthest_info());
		this->Thrad_life_local_copy.push_back(true);
	}

#pragma omp parallel \
num_threads(this->Threads_number)
	{
		int th_id = omp_get_thread_num();
		if (th_id == 0) {

			this->__Compute_new_Convex_Hull_Master_thread(max_iterations);

			this->Threads_life = false;
#pragma omp barrier
		}
		else {
			auto it_life = this->Thrad_life_local_copy.begin();
			advance(it_life, th_id);
			while (true) {
				this->__Recompute_farthest_thread(th_id);
				if (!(*it_life)) break;
			}

		}

	}

#else

	this->__Compute_new_Convex_Hull_Master_thread(max_iterations);

#endif

}

template<typename V>
void Fast_QHull<V>::Get_Incidences(std::list<std::list<size_t>>* indexes)  const {

	indexes->clear();
	if (this->Facets.empty()) {
#ifdef USE_CRASHING_ABORT
		abort();
#else
		return;
#endif
	}

	for (auto it = this->Facets.begin(); it != this->Facets.end(); it++) 
		indexes->push_back({ it->pos_A , it->pos_B , it->pos_C });

}

template<typename V>
size_t* Fast_QHull<V>::Get_Incidences(size_t* numb_of_facets)  const {

	if (this->Facets.empty()) {
#ifdef USE_CRASHING_ABORT
		abort();
#else
		return;
#endif
	}

	size_t* indexes = (size_t*)malloc(sizeof(size_t)*this->Facets.size() * 3);

	*numb_of_facets = this->Facets.size();
	size_t k = 0;
	for (auto it = this->Facets.begin(); it != this->Facets.end(); it++) {
		indexes[k] = it->pos_A;
		indexes[k+1] = it->pos_B;
		indexes[k+2] = it->pos_C;
		k += 3;
	}
	return indexes;

}

template<typename V>
void Fast_QHull<V>::Get_Normals(std::list<V>* normals)  const {

	normals->clear();
	if (this->Facets.empty()) {
#ifdef USE_CRASHING_ABORT
		abort();
#else
		return;
#endif
	}

	for (auto it = this->Facets.begin(); it != this->Facets.end(); it++)
		normals->push_back(V(it->N[0], it->N[1], it->N[2]));

}

#endif 
